// see https://cycling74.com/forums/topic/error-126-loading-external/#.WIvJXhsrKHs
#define MAXAPI_USE_MSCRT

// a bunch of likely Max includes:
extern "C" {
#include "ext.h"
#include "ext_obex.h"
#include "ext_dictionary.h"
#include "ext_dictobj.h"
#include "ext_systhread.h"
	
#include "z_dsp.h"
	
#include "jit.common.h"
#include "jit.vecmath.h"
#include "jit.gl.h"
#include "max.jit.mop.h"
}

#include "al_math.h"

/*

 Main capabilities of Steam Audio:
 
 NON-ENVIRONMENTAL:
 
 panners:
 
 mono -> hrtf
	iplCreateBinauralEffect + direction
 mono -> speaker
 mono -> ambi
	iplCreatePanningEffect + direction
 speaker -> ambi? (not sure if this is included)
	iplCreatePanningEffect + direction
 
 translators:
 
 ambi -> hrtf
	iplCreateAmbisonicsBinauralEffect
 ambi -> speaker
	iplCreateAmbisonicsPanningEffect
 speaker -> hrtf
	iplCreateVirtualSurroundEffect
 ambi rotate
	iplCreateAmbisonicsRotator
 
 
 ENVIRONMENTAL
 
 creating a scene
 
	one or more materials, one or more meshes, each triangle has a material ID
	
 
 direct sound (direction independent) fx:
 
	iplCreateDirectSoundEffect()
		get parameters (direction, distance atten, absorp, delay, occlusion, transmission factors):
		iplGetDirectSoundPath(env_renderer, listener pose, source pos + radius, settings)
		iplApplyDirectSoundEffect()
		seems designed for use with mono sources, but maybe it can also apply to virtual speakers?
 
 convolution fx:
 
	direction-dependent convolution reverb encoded in ambisonics
	
	iplCreateConvolutionEffect()
		
		put source in:
			iplSetDryAudioForConvolutionEffect() + position
		get reverb out:
			per source: iplGetWetAudioForConvolutionEffect()
			all sources: iplGetMixedEnvironmentalAudio()
 
  baking & probes:
	not sure if this is something useful for Max?
	if there's a desire to use baked rendering (lower cpu but higher memory cost)
    probes are needed to bake
 
 Generally:
 
 each source mono sound goes through a direct sound fx then an hrtf or speaker layout
 and also goes through a convolution mixer to an ambisonics bus that is decoded to hrtf or speaker layout
 docs also suggest mixing all sources into another convolution located at the listener position
 (i.e. all sources go into two reverbs)
 
*/
#include "steamaudio_api/include/phonon.h"

#include <new> // for in-place constructor
#include <vector>

static t_class * vrmsp_class = 0;

// Static state (shared by all):
struct VRMSP_Global {
	
	IPLContext context;
	IPLRenderingSettings settings;
	
	IPLAudioFormat mono_format;
	
	IPLHrtfParams hrtf_params;
	IPLAudioFormat hrtf_format;
	IPLhandle binaural_renderer = 0;
	
	// Only required if using Radeon Rays for ray tracing, or if using TrueAudio Next for convolution:
	IPLhandle computeDevice = NULL;
	IPLSimulationSettings simulationSettings;
	IPLhandle scene = 0;
	IPLhandle environment = 0;
	IPLhandle environmental_renderer = 0;
	
	VRMSP_Global() {
		
		// TODO: consider mapping to sysmem handers?
		context.allocateCallback = 0;
		context.freeCallback = 0;
		context.logCallback = log_callback;
		
		// use default convolution setting, as the alternative depends on AMD gpus
		settings.convolutionType = IPL_CONVOLUTIONTYPE_PHONON;
		// initialize with zero to make sure that setup() allocates objects:
		settings.frameSize = 0;
		settings.samplingRate = 0;
		
		mono_format.channelLayoutType	= IPL_CHANNELLAYOUTTYPE_SPEAKERS;
		mono_format.channelLayout		= IPL_CHANNELLAYOUT_MONO;
		mono_format.numSpeakers			= 1;
		mono_format.channelOrder		= IPL_CHANNELORDER_INTERLEAVED;
		
		hrtf_format.channelLayoutType	= IPL_CHANNELLAYOUTTYPE_SPEAKERS;
		hrtf_format.channelLayout		= IPL_CHANNELLAYOUT_STEREO;
		hrtf_format.numSpeakers			= 2;
		hrtf_format.channelOrder		= IPL_CHANNELORDER_INTERLEAVED;

		// TODO attrify, esp. maxConvolutionSources and ambisonicsOrder
		// TODO if any of these change, need to run setup_scene and setup_environment again
		// which in turn may require other vr~ objects to rebuild
		// (maybe dsp chain broken?) -> dspchain_setbroken(dspchain_get());
		simulationSettings.sceneType = IPL_SCENETYPE_PHONON; // other options available
		simulationSettings.numRays = 1024*32; //typical values are in the range of 1024 to 1024*128
		simulationSettings.numDiffuseSamples = 32*32; //typical values are in the range of 32 to 32*128
		simulationSettings.numBounces = 8; //typical values are in the range of 1 to 32.
		simulationSettings.irDuration = 2; //typical values are in the range of 0.5 to 4.0
		simulationSettings.ambisonicsOrder = 2; // typical values are between 1 and 3
		simulationSettings.maxConvolutionSources = 32; // max no. of sound sources
	}
	
	~VRMSP_Global() {
		cleanup();
	}
	
	void cleanup() {
		if (binaural_renderer) iplDestroyBinauralRenderer(&binaural_renderer);
		if (environment) iplDestroyEnvironment(&environment);
		if (environmental_renderer) iplDestroyEnvironmentalRenderer(&environmental_renderer);
		if (scene) iplDestroyScene(&scene);
	}
	
	static void log_callback(char* message) {
		object_post(0, "vr~: %s", message);
	}
	
	static void finalization_progress_callback(IPLfloat32 progress) {
		object_post(0, "vr~: scene finalized %d%%", int(progress * 100));
	}
	
	void setup(double samplerate, int framesize) {
		
		if (samplerate != 24000 && samplerate != 44100 && samplerate != 48000) {
			object_error(0, "vr~: unsupported samplerate %f; only 24000 Hz, 44100 Hz, and 48000 Hz are supported", samplerate);
			return;
		}
		if (framesize > 4096) {
			object_error(0, "vr~: unsupported vectorsize %d; up to 4096 supported", framesize);
			return;
		}
		
		// various options:
		hrtf_params.type = IPL_HRTFDATABASETYPE_DEFAULT; // or CUSTIOM
		hrtf_params.hrtfData = 0;	// Must be NULL.
		
		// TODO: allow custom HRTFs; implement these:
		hrtf_params.numHrirSamples = 0;
		hrtf_params.loadCallback = 0;
		hrtf_params.unloadCallback = 0;
		hrtf_params.lookupCallback = 0;
		
		if (settings.frameSize != framesize || settings.samplingRate != samplerate) {
			
			settings.samplingRate = samplerate;
			settings.frameSize = framesize;
			
			// trash any existing objects before creating:
			cleanup();
			
			// binaural
			if (IPL_STATUS_SUCCESS != iplCreateBinauralRenderer(context, settings, hrtf_params, &binaural_renderer)) {
				object_error(0, "vr~: failed to create binaural renderer");
				return;
			}
			
			setup_environment();
		}
	}
	
	// by default scene is NULL and that is fine
	// but at some point we may add support for loading scenes for lovely physically-based effects
	void setup_scene() {
		
		// work on a local copy until it is ready:
		IPLhandle temp_scene = 0;
		
		// how many different kinds of surface materials to support
		// probably this needs to load from some config?
		IPLint32 numMaterials = 0;
		// TODO: enable this at some point
		if (numMaterials > 0 && IPL_STATUS_SUCCESS != iplCreateScene(context,
																	 computeDevice,
																	 simulationSettings,
																	 numMaterials,
																	 &temp_scene)) {
			object_error(0, "vr~: failed to create scene");
			return;
		} else {
			
			// define numMaterials materials with iplSetSceneMaterial
			for (int i=0; i<numMaterials; i++) {
				IPLMaterial material;
				// set properties
				// materials defined in terms of absorption & transmittance at low/med/hi freqs, and also roughness
				iplSetSceneMaterial(temp_scene, i, material);
			}
			
			IPLint32 numVertices = 0, numTriangles = 0;
			IPLhandle staticMesh;
			if (IPL_STATUS_SUCCESS != iplCreateStaticMesh(temp_scene,
														  numVertices,
														  numTriangles,
														  &staticMesh)) {
				object_error(0, "vr~: failed to create mesh");
			} else {
				
				// can be one or multiple meshes.
				// can imagine creating this via a @matrixoutput of a jit.gl.mesh etc.
				// except not sure how to specify materials... by colours?
				// or maybe for simplicity, one material per mesh?
				
				// each mesh has to set:
				//	a list of numVertices vertices (vec3),				iplSetStaticMeshVertices
				//	a list of numTriangles triangles (3 indices),		iplSetStaticMeshTriangles
				//	a list of numTriangles materials (1 per triangle)	iplSetStaticMeshMaterials
				
				// done:
				iplDestroyStaticMesh(&staticMesh);
			}
			
			
			// finally:
			// NOTE: This is a time-consuming, blocking call, so do not call it from performance-sensitive code
			// i.e. setup_scene should be under defer()
			// if not in a differen thread altogether?
			iplFinalizeScene(temp_scene, finalization_progress_callback);
			
			// once done,
			// can trash the old one & rebuild:
			if (scene) iplDestroyScene(&scene);
			scene = temp_scene;
			
			// now trigger setup_environment...
			setup_environment();
			
			// TODO
			// NOTE: can also load/save scenes via iplSaveFinalizedScene/iplLoadFinalizedScene
			// NOTE: can also export to obj: iplDumpSceneToObjFile
		}

	}
	
	// TODO: this can happen at any time
	// because of setup_scene()
	// so any other objects refering to the environmental_renderer will also need rebuilding...
	void setup_environment() {
		IPLhandle probeManager = NULL; // ok to leave null if not using baked data
		IPLSimulationThreadCreateCallback threadCreateCallback = 0;
		IPLSimulationThreadDestroyCallback threadDestroyCallback = 0;
		
		// TODO: configure this
		// not sure if it wants to be the same as hrtf_format?
		IPLAudioFormat output_format;
		
		
		if (IPL_STATUS_SUCCESS != iplCreateEnvironment(context,
													   computeDevice,
													   simulationSettings,
													   scene,
													   probeManager,
													   &environment)) {
			object_error(0, "vr~: failed to create environment");
		} else if (IPL_STATUS_SUCCESS != iplCreateEnvironmentalRenderer(context,
																		environment,
																		settings,
																		hrtf_format, //output_format,
																		threadCreateCallback,
																		threadDestroyCallback,
																		&environmental_renderer)) {
			object_error(0, "vr~: failed to create environmental renderer");
		} else {
			// ok!
		}
	}
	
	// TODO: attr setter to call iplSetNumBounces(IPLhandle environment, IPLint32 numBounces);
	
} global;

/*
void ambi2hrtf() {
	
	IPLAudioFormat inputFormat; // must be ambisonic
	inputFormat.channelLayoutType = IPL_CHANNELLAYOUTTYPE_AMBISONICS;
	inputFormat.numSpeakers = 4; // TODO: depends on ambi type
	inputFormat.ambisonicsOrder = 1;
	inputFormat.ambisonicsOrdering = IPL_AMBISONICSORDERING_FURSEMALHAM; // or IPL_AMBISONICSORDERING_ACN;
	inputFormat.ambisonicsNormalization = IPL_AMBISONICSNORMALIZATION_FURSEMALHAM; // or IPL_AMBISONICSNORMALIZATION_SN3D or IPL_AMBISONICSNORMALIZATION_N3D;
	inputFormat.channelOrder = IPL_CHANNELORDER_INTERLEAVED; // or IPL_CHANNELORDER_DEINTERLEAVED
	
	// standard HRTF audio format 
	IPLAudioFormat outputFormat;
	outputFormat.channelLayoutType = IPL_CHANNELLAYOUTTYPE_SPEAKERS;
	outputFormat.channelLayout = IPL_CHANNELLAYOUT_STEREO;
	outputFormat.numSpeakers = 2;
	outputFormat.channelOrder = IPL_CHANNELORDER_INTERLEAVED;
	
	IPLhandle ambisonic_hrtf_effect = 0;
	if (IPL_STATUS_SUCCESS != iplCreateAmbisonicsBinauralEffect(global.binaural_renderer,
																inputFormat,
																global.hrtf_format,
																&ambisonic_hrtf_effect)) {
		object_error(0, "failed to create Ambisonics HRTF effect");
		return;
	}
}

void conv() {

	// phonon_environmental_renderer needs to exist
	// typically expects to use data baked from a game engine
	// but a more expensive REALTIME option is available
	// not sure what input/output types should be used
	// seems like generally want to create a convolution effect per source, since they are named?
	IPLhandle convolution_effect = 0;
	IPLstring name = "__reverb__"; // or a name baked from a game engine
	IPLSimulationType simulationType = IPL_SIMTYPE_REALTIME; // IPL_SIMTYPE_BAKED
	IPLAudioFormat inputFormat; // e.g. ambisonic
	IPLAudioFormat outputFormat; // e.g. ambisonic
	if (IPL_STATUS_SUCCESS != iplCreateConvolutionEffect(global.environmental_renderer,
														 name,
														 simulationType,
														 inputFormat,
														 outputFormat,
														 &convolution_effect)) {
		object_error(0, "failed to create convolution effect");
		return;
	}
	
	// audio loop
	{
		IPLVector3 sourcePosition; // world-space position of source
		IPLAudioBuffer dryAudio; // input
		iplSetDryAudioForConvolutionEffect(convolution_effect,
										   sourcePosition,
										   dryAudio);
		
 // for one source immediately:
		IPLVector3 listenerPosition;
		IPLVector3 listenerAhead; // unit forward vector of listener orientation,
		IPLVector3 listenerUp; // unit up vector of listener
		IPLAudioBuffer wetAudio; // output
		iplGetWetAudioForConvolutionEffect(convolution_effect,
										   listenerPosition,
										   listenerAhead,
										   listenerUp,
										   wetAudio);
 
		// more efficient: for all sources
		// but for this to work, it would need to be a separate object that comes after all sources in the dsp chain
		// (otherwise there'd be a buffer of latency for some sources)
		IPLhandle renderer;
		IPLVector3 listenerPosition; // world-space
		IPLVector3 listenerAhead, listenerUp; // unit vectors from listener orientation
		IPLAudioBuffer mixedWetAudio;
		iplGetMixedEnvironmentalAudio(global.environmental_renderer,
									  listenerPosition,
									  listenerAhead,
									  listenerUp,
									  mixedWetAudio);
	}
}
*/


class VRMSP_Mono_HRTF {
public:
	t_pxobject ob; // max objExamplet, must be first!
	
	// attr
	t_atom_long interp = 1;
	glm::vec3 direction;
	
	IPLhandle binaural = 0;
	// pre-allocated to maximum vector size, in case this is cheaper?
	IPLfloat32 source_buffer[4096];
	IPLfloat32 output_buffer[4096 * 2];
	
	VRMSP_Mono_HRTF() {
		// mono input:
		dsp_setup(&ob, 1);
		// stereo output:
		outlet_new(&ob, "signal");
		outlet_new(&ob, "signal");
		
		// default position in front of listener, to avoid 0,0,0
		direction.x = 0;
		direction.y = 0;
		direction.z = -1;
	}

	~VRMSP_Mono_HRTF() {
		cleanup();
	}
	
	void cleanup() {
		if (binaural) iplDestroyBinauralEffect(&binaural);
	}

	void dsp64(t_object *dsp64, short *count, double samplerate, long framesize, long flags) {
		global.setup(samplerate, framesize);
		
		// reset:
		cleanup();
		
		// create binaural effect:
		iplCreateBinauralEffect(global.binaural_renderer, global.mono_format, global.hrtf_format, &binaural);
		
		// connect to MSP dsp chain:
		long options = 0;
		object_method(dsp64, gensym("dsp_add64"), this, vrmsp_perform64, options, 0);
	}
	
	static void vrmsp_perform64(VRMSP_Mono_HRTF *x, t_object *dsp64, double **ins, long numins, double **outs, long numouts, long sampleframes, long flags, void *userparam) {
		x->perform64(dsp64, ins, numins, outs, numouts, sampleframes, flags);
	}
	
	void perform64(t_object *dsp64, double **ins, long numins, double **outs, long numouts, long sampleframes, long flags) {
		// phonon uses float32 processing, so we need to copy :-(
		IPLAudioBuffer outbuffer;
		outbuffer.format = global.hrtf_format;
		outbuffer.numSamples = sampleframes;
		outbuffer.interleavedBuffer = output_buffer;
		
		IPLAudioBuffer inbuffer;
		inbuffer.format = global.mono_format;
		inbuffer.numSamples = sampleframes;
		inbuffer.interleavedBuffer = source_buffer;
		
		// copy input:
		{
			t_double * src = ins[0];
			IPLfloat32 * dst = source_buffer;
			int n = sampleframes;
			while (n--) { *dst++ = *src++; }
		}
		
		// Unit vector from the listener to the point source,
		// relative to the listener's coordinate system.
		// TODO: apply head tracking quat transform to direction
		glm::vec3 dirn = glm::normalize(direction);
		
		// Note:
		// IPL_HRTFINTERPOLATION_BILINEAR has high CPU cost
		// Typically, bilinear filtering is most useful for wide-band noise-like sounds, such as radio static, mechanical noise, fire, etc.
		// BUT Must use IPL_HRTFINTERPOLATION_BILINEAR if using a custom HRTF
		
		{
			IPLAudioBuffer outbuffer;
			outbuffer.format = global.hrtf_format;
			outbuffer.numSamples = sampleframes;
			outbuffer.interleavedBuffer = output_buffer;
			iplApplyBinauralEffect(binaural,
								   inbuffer,
								   *(IPLVector3 *)(&dirn.x),
								   interp ? IPL_HRTFINTERPOLATION_BILINEAR : IPL_HRTFINTERPOLATION_NEAREST,
								   outbuffer);
		}
		
		// copy output:
		{
			IPLfloat32 * src = output_buffer;
			t_double * dst0 = outs[0];
			t_double * dst1 = outs[1];
			int n = sampleframes;
			while (n--) {
				*dst0++ = *src++;
				*dst1++ = *src++;
			}
		}
	}
};

void * vrmsp_new(t_symbol *s, long argc, t_atom *argv) {
	VRMSP_Mono_HRTF *x = NULL;
	if ((x = (VRMSP_Mono_HRTF *)object_alloc(vrmsp_class))) {
		x = new (x) VRMSP_Mono_HRTF();
		attr_args_process(x, (short)argc, argv);
	}
	return (x);
}

void vrmsp_free(VRMSP_Mono_HRTF *x) {
	x->~VRMSP_Mono_HRTF();
}

// registers a function for the signal chain in Max
void vrmsp_dsp64(VRMSP_Mono_HRTF *x, t_object *dsp64, short *count, double samplerate, long maxvectorsize, long flags) {
	x->dsp64(dsp64, count, samplerate, maxvectorsize, flags);
}

void vrmsp_assist(VRMSP_Mono_HRTF *x, void *b, long m, long a, char *s) {
	if (m == ASSIST_INLET) {
		sprintf(s, "source (signal)");
	} else {
		switch(a) {
			case 0: sprintf(s, "headphone left (signal)"); break;
			case 1: sprintf(s, "headphone right (signal)"); break;
		}
	}
}

void vrmsp_init() {
	t_class * c = class_new("vr~", (method)vrmsp_new, (method)vrmsp_free, (long)sizeof(VRMSP_Mono_HRTF), 0L, A_GIMME, 0);
	class_addmethod(c, (method)vrmsp_assist, "assist", A_CANT, 0);
	class_addmethod(c, (method)vrmsp_dsp64, "dsp64", A_CANT, 0);
	
	CLASS_ATTR_LONG(c, "interp", 0, VRMSP_Mono_HRTF, interp);
	CLASS_ATTR_STYLE(c, "interp", 0, "onoff");
	CLASS_ATTR_FLOAT_ARRAY(c, "direction", 0, VRMSP_Mono_HRTF, direction, 3);
	
	class_dspinit(c);
	class_register(CLASS_BOX, c);
	vrmsp_class = c;
}


void ext_main(void *r) {
	
	vrmsp_init();
}
