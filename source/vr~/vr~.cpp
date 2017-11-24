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

static t_class * static_hrtf_class = 0;
static t_class * static_ambi2hrtf_class = 0;

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
		mono_format.channelOrder		= IPL_CHANNELORDER_DEINTERLEAVED;
		
		hrtf_format.channelLayoutType	= IPL_CHANNELLAYOUTTYPE_SPEAKERS;
		hrtf_format.channelLayout		= IPL_CHANNELLAYOUT_STEREO;
		hrtf_format.numSpeakers			= 2;
		hrtf_format.channelOrder		= IPL_CHANNELORDER_DEINTERLEAVED;
		
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
			
			// trash any existing objects before creating:
			cleanup();
			
			// settings have changed:
			settings.frameSize = framesize;
			settings.samplingRate = samplerate;
			
			// binaural
			if (IPL_STATUS_SUCCESS != iplCreateBinauralRenderer(context, settings, hrtf_params, &binaural_renderer)) {
				object_error(0, "vr~: failed to create binaural renderer");
				return;
			}
			
			//setup_environment();
			
			object_post(0, "vr~ changed global settings %d %f\n", framesize, samplerate);
			
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
		//IPLAudioFormat output_format;
		
		
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

struct VRMSP_ambi2hrtf {
	t_pxobject ob;
	// attrs:
	//t_atom_long normalization = 0;
	//t_atom_long channelorder = 0;
	// A unit quaternion describing the 3D transformation from world space to listener space coordinates.
	//glm::quat orientation, orientation_prev;
	
	IPLAudioFormat ambisonic_format; // must be ambisonic
	IPLhandle ambisonic_hrtf_effect = 0;
	IPLhandle ambisonic_rotator = 0;
	
	// pre-allocated to maximum vector size, in case this is cheaper?
	IPLfloat32 * ambi_buffers[16];
	IPLfloat32 * output_buffers[2];
	
	VRMSP_ambi2hrtf() {
		// according to https://github.com/ValveSoftware/steam-audio/issues/38
		// the only supported combination is IPL_CHANNELORDER_DEINTERLEAVED,
		// IPL_AMBISONICSORDERING_ACN, and IPL_AMBISONICSNORMALIZATION_N3D for the input buffer
		
		ambisonic_format.channelLayoutType = IPL_CHANNELLAYOUTTYPE_AMBISONICS;
		ambisonic_format.numSpeakers = 16; //4; // TODO: depends on ambi type
		ambisonic_format.channelOrder = IPL_CHANNELORDER_DEINTERLEAVED;
		ambisonic_format.ambisonicsOrder = 3; //1;
		ambisonic_format.ambisonicsOrdering = IPL_AMBISONICSORDERING_ACN;
		ambisonic_format.ambisonicsNormalization = IPL_AMBISONICSNORMALIZATION_N3D;
		
		for (int i=0; i<ambisonic_format.numSpeakers; i++) {
			ambi_buffers[i] = new float[4096];
		}
		for (int i=0; i<2; i++) {
			output_buffers[i] = new float[4096];
		}
		
		//orientation[0] = orientation[1] = orientation[2] = 0;
		//orientation[3] = 1;
		
		// signal inlets:
		dsp_setup(&ob, ambisonic_format.numSpeakers);
		
		// stereo output:
		outlet_new(&ob, "signal");
		outlet_new(&ob, "signal");
	}
	
	~VRMSP_ambi2hrtf() {
		cleanup();
		
		for (int i=0; i<ambisonic_format.numSpeakers; i++) {
			delete[] ambi_buffers[i];
		}
		for (int i=0; i<2; i++) {
			delete[] output_buffers[i];
		}
	}
	
	void cleanup() {
		if (ambisonic_hrtf_effect) iplDestroyAmbisonicsBinauralEffect(&ambisonic_hrtf_effect);
		if (ambisonic_rotator) iplDestroyAmbisonicsRotator(&ambisonic_rotator);
	}
	
	void dsp64(t_object *dsp64, short *count, double samplerate, long framesize, long flags) {
		global.setup(samplerate, framesize);
		
		// reset:
		cleanup();
		
		// create binaural effect:
		/*
		 switch (channelorder) {
			case 1: input_format.ambisonicsOrdering = IPL_AMBISONICSORDERING_ACN; break;
			default: input_format.ambisonicsOrdering = IPL_AMBISONICSORDERING_FURSEMALHAM; break;
		 }
		 switch (normalization) {
			case 2: input_format.ambisonicsNormalization = IPL_AMBISONICSNORMALIZATION_FURSEMALHAM; break;
			case 1: input_format.ambisonicsNormalization = IPL_AMBISONICSNORMALIZATION_SN3D; break;
			default: input_format.ambisonicsNormalization = IPL_AMBISONICSNORMALIZATION_N3D; break;
		 }
		 */
		
		if (IPL_STATUS_SUCCESS != iplCreateAmbisonicsBinauralEffect(global.binaural_renderer,
																	ambisonic_format,
																	global.hrtf_format,
																	&ambisonic_hrtf_effect)) {
			object_error(0, "failed to create Ambisonics HRTF effect");
			return;
		}
		
		if (IPL_STATUS_SUCCESS != iplCreateAmbisonicsRotator(ambisonic_format.ambisonicsOrder, &ambisonic_rotator)) {
			object_error(0, "failed to create Ambisonics rotator effect");
			return;
		}
		
		// connect to MSP dsp chain:
		long options = 0;
		object_method(dsp64, gensym("dsp_add64"), this, static_perform64, options, 0);
	}
	
	void perform64(t_object *dsp64, double **ins, long numins, double **outs, long numouts, long sampleframes, long flags) {
		// phonon uses float32 processing, so we need to copy :-(
		
		IPLAudioBuffer inbuffer;
		inbuffer.format = ambisonic_format;
		inbuffer.numSamples = sampleframes;
		inbuffer.deinterleavedBuffer = ambi_buffers;
		
		// copy input:
		for (int i=0; i<ambisonic_format.numSpeakers; i++) {
			t_double * src = ins[i];
			IPLfloat32 * dst = ambi_buffers[i];
			int n = sampleframes;
			while (n--) {
				*dst++ = *src++;
			}
		}
		
		/*
		 // The steam audio implementation doesn't seem to work at all
		 // see https://github.com/ValveSoftware/steam-audio/issues/38
		 
		 IPLQuaternion q; // xyzw format. jitter also uses xyzw format, which is handy:
		 q.x = orientation[0];
		 q.y = orientation[1];
		 q.z = orientation[2];
		 q.w = orientation[3];
		 iplSetAmbisonicsRotation(ambisonic_rotator, q);
		 
		 object_post(0, "q %f %f %f %f", q.x, q.y, q.z, q.w);
		 
		 // TODO: It is possible to pass the same value for \c inputAudio and \c outputAudio.
		 // This results in in-place rotation of the Ambisonics data.
		 iplRotateAmbisonicsAudioBuffer(ambisonic_rotator, inbuffer, rotbuffer);
		 
		 
		 // rotate input:
		 // TODO: make the slerp interpolation independent of framesize
		 if (0) {
			// in-place rotation:
			IPLfloat32 * dst = source_buffer;
			IPLfloat32 * src = source_buffer;
		 
			int n = sampleframes;
			float div = 1.f/sampleframes;
			while (n--) {
		 src++; // W unused, but need to increment pointer
		 float x = *src++;
		 float y = *src++;
		 float z = *src++;
		 
		 float a = n*div; // slides from 1 to 0
		 glm::quat slerped = glm::slerp(orientation, orientation_prev, a);
		 
		 // or inverse?
		 //glm::mat4 m = glm::mat4_cast(glm::inverse(orientation));
		 //glm::vec4 v = m * glm::vec4(x, y, z, 1.f);
		 
		 // TODO: rotate or unrotate?
		 glm::vec3 dir(x, y, z);
		 glm::vec3 v = quat_rotate(slerped, dir);
		 
		 dst++; // W unused, but need to incremment pointer
		 *dst++ = v.x;
		 *dst++ = v.y;
		 *dst++ = v.z;
			}
		 }
		 orientation_prev = orientation;
		 */
		
		IPLAudioBuffer outbuffer;
		outbuffer.format = global.hrtf_format;
		outbuffer.numSamples = sampleframes;
		outbuffer.deinterleavedBuffer = output_buffers;
		
		iplApplyAmbisonicsBinauralEffect(ambisonic_hrtf_effect, inbuffer, outbuffer);
		
		// copy output:
		{
			IPLfloat32 * src0 = output_buffers[0];
			IPLfloat32 * src1 = output_buffers[1];
			t_double * dst0 = outs[0];
			t_double * dst1 = outs[1];
			int n = sampleframes;
			while (n--) {
				*dst0++ = *src0++;
				*dst1++ = *src1++;
			}
		}
	}
	
	///////////////
	
	static void * static_new(t_symbol *s, long argc, t_atom *argv) {
		VRMSP_ambi2hrtf *x = NULL;
		if ((x = (VRMSP_ambi2hrtf *)object_alloc(static_ambi2hrtf_class))) {
			x = new (x) VRMSP_ambi2hrtf();
			attr_args_process(x, (short)argc, argv);
		}
		return (x);
	}
	
	static void static_free(VRMSP_ambi2hrtf *x) {
		x->~VRMSP_ambi2hrtf();
	}
	
	// registers a function for the signal chain in Max
	static void static_dsp64(VRMSP_ambi2hrtf *x, t_object *dsp64, short *count, double samplerate, long maxvectorsize, long flags) {
		x->dsp64(dsp64, count, samplerate, maxvectorsize, flags);
	}
	
	static void static_perform64(VRMSP_ambi2hrtf *x, t_object *dsp64, double **ins, long numins, double **outs, long numouts, long sampleframes, long flags, void *userparam) {
		x->perform64(dsp64, ins, numins, outs, numouts, sampleframes, flags);
	}
	
	static void static_assist(VRMSP_ambi2hrtf *x, void *b, long m, long a, char *s) {
		if (m == ASSIST_INLET) {
			sprintf(s, "ambisonic source (signal)");
		} else {
			switch(a) {
				case 0: sprintf(s, "headphone left (signal)"); break;
				case 1: sprintf(s, "headphone right (signal)"); break;
			}
		}
	}
	
	static void static_init() {
		t_class * c = class_new("vr.ambi2hrtf~", (method)VRMSP_ambi2hrtf::static_new, (method)VRMSP_ambi2hrtf::static_free, (long)sizeof(VRMSP_ambi2hrtf), 0L, A_GIMME, 0);
		class_addmethod(c, (method)VRMSP_ambi2hrtf::static_assist, "assist", A_CANT, 0);
		class_addmethod(c, (method)VRMSP_ambi2hrtf::static_dsp64, "dsp64", A_CANT, 0);
		
		//CLASS_ATTR_LONG(c, "normalization", 0, VRMSP_ambi2hrtf, normalization);
		//CLASS_ATTR_ENUMINDEX3(c, "normalization", 0, "n3d", "sn3d", "fuma");
		//CLASS_ATTR_LONG(c, "channelorder", 0, VRMSP_ambi2hrtf, channelorder);
		//CLASS_ATTR_ENUMINDEX2(c, "channelorder", 0, "fuma", "acn");
		
		//CLASS_ATTR_FLOAT_ARRAY(c, "quat", 0, VRMSP_ambi2hrtf, orientation, 4);
		
		class_register(CLASS_BOX, c);
		class_dspinit(c);
		static_ambi2hrtf_class = c;
	}
	
};


static t_class * VRMSP_hrtf_class = 0;

struct VRMSP_hrtf {
	t_pxobject ob;
	
	// attr
	t_atom_long interp;
	glm::vec3 direction;
	glm::quat quat; // the listener's head orientation
	
	IPLhandle binaural = 0;
	IPLfloat32 * source_buffers[1];
	IPLfloat32 * output_buffers[2];
	
	VRMSP_hrtf() {
		// pre-allocated to maximum vector size, in case this is cheaper?
		source_buffers[0] = new float[4096];
		output_buffers[0] = new float[4096];
		output_buffers[1] = new float[4096];
		
		// mono input:
		dsp_setup(&ob, 1);
		// stereo output:
		outlet_new(&ob, "signal");
		outlet_new(&ob, "signal");
		
		// attr defaults:
		interp = 1;
		// default position in front of listener, to avoid 0,0,0
		direction.x = 0;
		direction.y = 0;
		direction.z = -1;
		
		// default orientation has no transform:
		quat.x = 0;
		quat.y = 0;
		quat.z = 0;
		quat.w = 1;
	}
	
	~VRMSP_hrtf() {
		cleanup();
		
		delete[] source_buffers[0];
		delete[] output_buffers[0];
		delete[] output_buffers[1];
	}
	
	void cleanup() {
		if (binaural) iplDestroyBinauralEffect(&binaural);
	}
	
	void dsp64(t_object *dsp64, short *count, double samplerate, long framesize, long flags) {
		
		// reset:
		cleanup();
		global.setup(samplerate, framesize);
		
		//object_post((t_object *)this, "dsp %f %d interp %d", samplerate, framesize, interp);
		
		// create binaural effect:
		iplCreateBinauralEffect(global.binaural_renderer, global.mono_format, global.hrtf_format, &binaural);
		
		// connect to MSP dsp chain:
		long options = 0;
		object_method(dsp64, gensym("dsp_add64"), this, static_perform64, options, 0);
	}
	
	void perform64(t_object *dsp64, double **ins, long numins, double **outs, long numouts, long sampleframes, long flags) {
		
		// phonon uses float32 processing, so we need to copy :-(
		IPLAudioBuffer outbuffer;
		outbuffer.format = global.hrtf_format;
		outbuffer.numSamples = sampleframes;
		outbuffer.deinterleavedBuffer = output_buffers;
		
		IPLAudioBuffer inbuffer;
		inbuffer.format = global.mono_format;
		inbuffer.numSamples = sampleframes;
		inbuffer.deinterleavedBuffer = source_buffers;
		
		// copy input:
		{
			t_double * src = ins[0];
			IPLfloat32 * dst = source_buffers[0];
			int n = sampleframes;
			while (n--) { *dst++ = *src++; }
		}
		
		// Unit vector from the listener to the point source,
		// relative to the listener's coordinate system.
		// TODO: apply head tracking quat transform to direction
		glm::vec3 dirn = glm::normalize(direction);
		
		// rotate into head orientation:
		dirn = quat_unrotate(quat, dirn); // quat_rotate or quat_unrotate?
		
		// Note:
		// IPL_HRTFINTERPOLATION_BILINEAR has high CPU cost
		// Typically, bilinear filtering is most useful for wide-band noise-like sounds, such as radio static, mechanical noise, fire, etc.
		// BUT Must use IPL_HRTFINTERPOLATION_BILINEAR if using a custom HRTF
		
		{
			IPLAudioBuffer outbuffer;
			outbuffer.format = global.hrtf_format;
			outbuffer.numSamples = sampleframes;
			outbuffer.deinterleavedBuffer = output_buffers;
			iplApplyBinauralEffect(binaural,
								   inbuffer,
								   *(IPLVector3 *)(&dirn.x),
								   interp ? IPL_HRTFINTERPOLATION_BILINEAR : IPL_HRTFINTERPOLATION_NEAREST,
								   outbuffer);
		}
		
		// copy output:
		{
			IPLfloat32 * src0 = output_buffers[0];
			IPLfloat32 * src1 = output_buffers[1];
			t_double * dst0 = outs[0];
			t_double * dst1 = outs[1];
			int n = sampleframes;
			while (n--) {
				*dst0++ = *src0++;
				*dst1++ = *src1++;
			}
		}
	}
	
	static void * create(t_symbol *s, long argc, t_atom *argv) {
		VRMSP_hrtf *x = NULL;
		if ((x = (VRMSP_hrtf *)object_alloc(VRMSP_hrtf_class))) {
			x = new (x) VRMSP_hrtf;
			attr_args_process(x, (short)argc, argv);
		}
		return (x);
	}
	
	static void destroy(VRMSP_hrtf *x) {
		x->~VRMSP_hrtf();
	}
	
	// registers a function for the signal chain in Max
	static void static_dsp64(VRMSP_hrtf *x, t_object *dsp64, short *count, double samplerate, long maxvectorsize, long flags) {
		x->dsp64(dsp64, count, samplerate, maxvectorsize, flags);
	}
	
	static void static_perform64(VRMSP_hrtf *x, t_object *dsp64, double **ins, long numins, double **outs, long numouts, long sampleframes, long flags, void *userparam) {
		x->perform64(dsp64, ins, numins, outs, numouts, sampleframes, flags);
	}
	
	static void static_assist(VRMSP_hrtf *x, void *b, long m, long a, char *s) {
		if (m == ASSIST_INLET) {
			sprintf(s, "source (signal)");
		} else {
			switch(a) {
				case 0: sprintf(s, "headphone left (signal)"); break;
				case 1: sprintf(s, "headphone right (signal)"); break;
			}
		}
	}
	
	static void static_init() {
		t_class * c = class_new("vr.hrtf~", (method)create, (method)destroy, (long)sizeof(VRMSP_hrtf), 0L, A_GIMME, 0);
		
		class_addmethod(c, (method)static_assist, "assist", A_CANT, 0);
		class_addmethod(c, (method)static_dsp64, "dsp64", A_CANT, 0);
		
		CLASS_ATTR_LONG(c, "interp", 0, VRMSP_hrtf, interp);
		CLASS_ATTR_STYLE(c, "interp", 0, "onoff");
		CLASS_ATTR_FLOAT_ARRAY(c, "direction", 0, VRMSP_hrtf, direction, 3);
		CLASS_ATTR_FLOAT_ARRAY(c, "quat", 0, VRMSP_hrtf, quat, 4);
		
		class_dspinit(c);
		class_register(CLASS_BOX, c);
		VRMSP_hrtf_class = c;
	}
};





extern "C" C74_EXPORT void ext_main(void *r) {
	
	VRMSP_hrtf::static_init();
	VRMSP_ambi2hrtf::static_init();
}
