// to avoid creating nauseating lag
// the normal loop should be:
// 		run simulation stuff to update state (ideally not in main thread)
//		*bang() to get HMD pose
// 		*capture scene (jit.gl.node @capture 1) for both eye cameras
// 		*send the texture to submit_texture_to_hmd()
// 		draw any desktop window stuff
// 		swap buffers (i.e. bang() jit.gl.render)
// the stuff with * symbols should happen as near as possible

#include "c74_jitter.h"

using namespace c74::max;

extern "C" {
	#include "jit.gl.h"

#ifdef WIN_VERSION
	// needed this for glFrameBuffer / GL_FRAMEBUFFER symbols
	// (and needed to comment out the jit.common.h include)
	#include "jit.gl.procs.h"
	#include "jit.gl.support.h"
#endif

	// needed to declare these here, as they aren't declared in c74_jitter.h:
	void * jit_object_findregistered(t_symbol *s);
	void * jit_object_register(void *x, t_symbol *s);
	void * jit_object_findregistered(c74::max::t_symbol *s);
	void * jit_object_register(void *x, c74::max::t_symbol *s);
}

// Oculus:
#include "OVR_CAPI.h"
#include "OVR_CAPI_GL.h"
// Vive:
// The OpenVR SDK:
#include "openvr.h"

#include "al_math.h"


static bool oculus_initialized = 0;

static t_symbol * ps_glid;
static t_symbol * ps_jit_gl_texture;
static t_symbol * ps_viewport;

// jitter uses xyzw format
// glm:: uses wxyz format
// xyzw -> wxyz
glm::quat quat_from_jitter(glm::quat const & v) {
	return glm::quat(v.z, v.w, v.x, v.y);
}

/*
glm::quat quat_from_jitter(t_jit_quat const & v) {
	return glm::quat(v.w, v.x, v.y, v.z);
}*/

// wxyz -> xyzw
glm::quat quat_to_jitter(glm::quat const & v) {
	return glm::quat(v.x, v.y, v.z, v.w);
}




#include <string>
#include <fstream>


static t_class* this_class = nullptr;

struct Vr {
	t_object ob;
	void * ob3d;
	void * outlet_msg;
	//void * outlet_video;
	void * outlet_tracking;
	//void * outlet_node;
	void * outlet_eye[2];
	//void * outlet_controller[2];
	void * outlet_tex;
	
	// attrs:
	float near_clip = 0.15f;
	float far_clip = 100.f;
	
	// configuration:
	int connected = 0;
	int dest_ready = 0;
	
	// guts:
	
	// FBO & texture that the scene is copied into
	// (we can't submit the jit_gl_texture directly)
	GLuint fbo_id, rbo_id, fbo_texture_id;
	t_atom_long fbo_texture_dim[2];

	// driver-specific:
	struct {
		ovrSession session;
		ovrSessionStatus status;
		ovrHmdDesc hmd;
		ovrGraphicsLuid luid;
		ovrEyeRenderDesc eyeRenderDesc[2];
		ovrVector3f      hmdToEyeViewOffset[2];
		ovrLayerEyeFov layer;
		ovrSizei pTextureDim;
		ovrTextureSwapChain textureChain;
		ovrMirrorTexture mirrorTexture;
	} oculus;
	
	Vr(t_symbol * drawto) {
		// init Max object:
		jit_ob3d_new(this, drawto);
		// outlets create in reverse order:
		outlet_msg = outlet_new(&ob, NULL);
		//outlet_video = outlet_new(&ob, "jit_gl_texture");
		//outlet_controller[1] = outlet_new(&ob, NULL);
		//outlet_controller[0] = outlet_new(&ob, NULL);
		outlet_tracking = outlet_new(&ob, NULL);
		//outlet_node = outlet_new(&ob, NULL);
		outlet_eye[1] = outlet_new(&ob, NULL);
		outlet_eye[0] = outlet_new(&ob, NULL);
		outlet_tex = outlet_new(&ob, "jit_gl_texture");
		
		// some whatever defaults, will get overwritten when driver connects
		fbo_texture_dim[0] = 1920;
		fbo_texture_dim[1] = 1080;
	}

	~Vr() {
		// free GL resources created by this external
		dest_closing();
		// disconnect from session
		disconnect();
		// remove from jit.gl* hierarchy
		jit_ob3d_free(this);
		// actually delete object
		max_jit_object_free(this);
	}
	
	// attempt to acquire the HMD
	void connect() {
		if (connected) disconnect();

		// try to see if the Oculus driver is available:
		if (!(oculus_initialized || oculusrift_init())) return;
		
		// TODO: driver-specific stuff
		if (!oculus_connect()) return;
		
		// if successful:
		connected = 1;
		
		configure();

		// if gpu is ready, go ahead & make what we need
		if (dest_ready) {
			create_gpu_resources();
		}
	}
	
	// release the HMD
	void disconnect() {
		if (!connected) return;

		// TODO: driver-specific stuff
		oculus_disconnect();
	
		connected = 0;
	}

	// called whenever HMD properties change
	// will dump a lot of information to the last outlet
	void configure() {
		t_atom a[6];
		
		if (connected) {
			// TODO get driver details & output
			// get recommended texture dim & output
			// initialize camera matrices, frusta, etc. & output
		}
		
		// now output camera properties:
		
		// set camera viewports:
		atom_setfloat(a + 0, 0.);
		atom_setfloat(a + 1, 0.);
		atom_setfloat(a + 2, 0.5);
		atom_setfloat(a + 3, 1.);
		outlet_anything(outlet_eye[0], ps_viewport, 4, a);
		atom_setfloat(a + 0, 0.5);
		outlet_anything(outlet_eye[1], ps_viewport, 4, a);
			
	}
	
	// Jitter GL context changed, need to reallocate GPU objects
	// happens on construction (if context already existed)
	// or when gl context is created
	// e.g. when creating jit.world or entering/leaving fullscreen
	t_jit_err dest_changed() {
		// mark drawto gpu context as usable
		// might not allocate gpu resources immediately, depends on whether driver is also connected
		dest_ready = 1;

		// try to connect:
		if (!connected) connect();

		if (connected) {
			create_gpu_resources();
		}
		return JIT_ERR_NONE;
	}

	// Jitter GL context closing, need to destroy GPU objects
	// happens on destruction of object (if context already existed)
	// or when gl context is destroyed
	// e.g. when deleting jit.world or entering/leaving fullscreen
	t_jit_err dest_closing() {

		// automatically disconnect at this point
		// since without a Jitter GPU context, there's nothing we can do
		// and disconnecting will free up the driver for other connections
		// e.g. as needed when entering/leaving fullscreen
		disconnect();

		// mark drawto gpu context as unusable
		dest_ready = 0;

		release_gpu_resources();

		return JIT_ERR_NONE;
	}

	void create_gpu_resources() {
		if (!connected && !dest_ready) return;

		// get drawto context:
		t_symbol *context = object_attr_getsym(this, gensym("drawto"));
		
		// create the FBO used to pass the scene texture to the driver:
		create_fbo(&fbo_id, &rbo_id, &fbo_texture_id, fbo_texture_dim);
		
		// TODO gpu resources for mirror, videocamera,
	}

	void release_gpu_resources() {
		// release associated resources:
		if (fbo_id) {
			glDeleteFramebuffersEXT(1, &fbo_id);
			fbo_id = 0;
		}
		if (fbo_texture_id) {
			glDeleteTextures(1, &fbo_texture_id);
			fbo_texture_id = 0;
		}
		if (rbo_id) {
			glDeleteRenderbuffersEXT(1, &rbo_id);
			rbo_id = 0;
		}

		// TODO gpu resources for mirror, videocamera, 
	}
	
	// poll HMD for events
	// most importantly, it picks up the current HMD pose
	// this should be the *last* thing to happen before rendering the scene
	// nothing time-consuming should happen between bang() and render
	void bang() {
		// if not connected, possibly, poll for availability?
		if (!connected) return;
		
		// get desired view matrix (from @position and @quat attrs)
		glm::vec3 m_position;
		//object_attr_getfloat_array(this, _jit_sym_position, 3, &m_position.x);
		object_attr_getfloat_array(this, _jit_sym_position, 3, &m_position.x);
		
		//t_jit_quat jitquat;
		glm::quat jitquat;
		//object_attr_getfloat_array(this, _jit_sym_quat, 4, &jitquat.x);
		object_attr_getfloat_array(this, _jit_sym_quat, 4, &jitquat.x);
		glm::mat4 modelview_mat = glm::translate(glm::mat4(1.0f), m_position) * mat4_cast(quat_from_jitter(jitquat));
		
		// TODO: video (or separate message for this?)
		
		// TODO: driver poll events
		// get tracking data
		// output headset & controllers, both in worldspace and trackingspace
		// output device states
		
		// compute camera poses & output to jit.gl.cameras 
		// (also output frusta, viewport, ?)
	}
	
	// triggered by "jit_gl_texture" message:
	// submit a texture received from Max to the HMD
	// the texture would typically be a captured jit.gl.node,
	// and should be a side-by-side stereo image 
	// from cameras calibrated by configure() and bang()
	void jit_gl_texture(t_symbol * intexture) {		
		void * jit_texture = jit_object_findregistered(intexture);
		if (!jit_texture) {
			object_error(&ob, "no texture to draw");
			return;	// no texture to copy from.
		}
		// TODO: verify that texob is a texture
		
		if (connected) {
			submit_texture_to_hmd(jit_texture);
		}
		
		t_atom a[1];
		if (connected) {
			// TODO: use mirror textures here if desired?
		}
		atom_setsym(a, intexture);
		outlet_anything(outlet_tex, ps_jit_gl_texture, 1, a);
	}
	
	void submit_texture_to_hmd(void * jit_texture) {
		//GLuint input_texture_id = object_attr_getlong(jit_texture, ps_glid);
		GLuint input_texture_id = object_attr_getlong(jit_texture, ps_glid);
		// get input texture dimensions
		t_atom_long input_texture_dim[2];
		object_attr_getlong_array(jit_texture, _jit_sym_dim, 2, input_texture_dim);
		//post("submit texture id %ld dim %ld %ld\n", input_texture_id, input_texture_dim[0], input_texture_dim[1]);
		
		if (!fbo_id) {
			// TODO try to allocate FBO for copying Jitter texture to driver?
			// or just bug out at this point?
		}
		
		// TODO: check success
		if (copy_texture_to_fbo(input_texture_id, input_texture_dim,
						 	    fbo_id, fbo_texture_id, fbo_texture_dim)) {
						 	
			// submit fbo to driver
		}
						 	
	}
	
	//////////////////////////////////////////////////////////////////////////////////////
	
	bool create_fbo(GLuint * fbo_id_ptr, GLuint * rbo_id_ptr, GLuint * fbo_texture_id_ptr, t_atom_long fbo_texture_dim[2]) {
		glGenFramebuffersEXT(1, fbo_id_ptr);
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, *fbo_id_ptr);
		
		glGenRenderbuffersEXT(1, rbo_id_ptr);
		glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, *rbo_id_ptr);
		glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT, fbo_texture_dim[0], fbo_texture_dim[1]);
		glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, *rbo_id_ptr);

		glGenTextures(1, fbo_texture_id_ptr);
		glBindTexture(GL_TEXTURE_2D, *fbo_texture_id_ptr);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, fbo_texture_dim[0], fbo_texture_dim[1], 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, *fbo_texture_id_ptr, 0);

		// check FBO status
		GLenum status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
		if (status != GL_FRAMEBUFFER_COMPLETE_EXT) {
			object_error(&ob, "failed to create Jitter FBO");
			return false;
		}

		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
		return true;
	}
	
	bool copy_texture_to_fbo(GLuint 		input_texture_id, 
							 t_atom_long 	input_texture_dim[2],
							 GLuint 		fbo_id, 
							 GLuint 		fbo_texture_id, 
							 t_atom_long 	fbo_texture_dim[2]) {
		// copy our glid source into the inFBO destination
		// save some state
		GLint previousFBO;	// make sure we pop out to the right FBO
		GLint previousMatrixMode;

		glGetIntegerv(GL_FRAMEBUFFER_BINDING_EXT, &previousFBO);
		glGetIntegerv(GL_MATRIX_MODE, &previousMatrixMode);

		// save texture state, client state, etc.
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glPushClientAttrib(GL_CLIENT_ALL_ATTRIB_BITS);

		// TODO use rectangle 1?
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo_id);
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, fbo_texture_id, 0);
		if (fbo_check()) {
			glMatrixMode(GL_TEXTURE);
			glPushMatrix();
			glLoadIdentity();

			glViewport(0, 0, fbo_texture_dim[0], fbo_texture_dim[1]);
			
			glMatrixMode(GL_PROJECTION);
			glPushMatrix();
			glLoadIdentity();
			glOrtho(0.0, fbo_texture_dim[0], 0.0, fbo_texture_dim[1], -1, 1);

			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glLoadIdentity();

			glClearColor(0, 0, 0, 1);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			glColor4f(1.0, 0.0, 1.0, 1.0);

			glActiveTexture(GL_TEXTURE0);
			glClientActiveTexture(GL_TEXTURE0);
			glEnable(GL_TEXTURE_RECTANGLE_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB, input_texture_id);
			
			// do not need blending if we use black border for alpha and replace env mode, saves a buffer wipe
			// we can do this since our image draws over the complete surface of the FBO, no pixel goes untouched.

			glDisable(GL_BLEND);
			glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

			// move to VA for rendering
			GLfloat tex_coords[] = {
				(GLfloat)input_texture_dim[0], (GLfloat)input_texture_dim[1],
				0.0, (GLfloat)input_texture_dim[1],
				0.0, 0.,
				(GLfloat)input_texture_dim[0], 0.
			};

			GLfloat verts[] = {
				(GLfloat)fbo_texture_dim[0], (GLfloat)fbo_texture_dim[1],
				0.0, (GLfloat)fbo_texture_dim[1],
				0.0, 0.0,
				(GLfloat)fbo_texture_dim[0], 0.0
			};
			
			glEnableClientState(GL_TEXTURE_COORD_ARRAY);
			glTexCoordPointer(2, GL_FLOAT, 0, tex_coords);
			glEnableClientState(GL_VERTEX_ARRAY);
			glVertexPointer(2, GL_FLOAT, 0, verts);
			glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
			glDisableClientState(GL_VERTEX_ARRAY);
			glDisableClientState(GL_TEXTURE_COORD_ARRAY);

			glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);

			glMatrixMode(GL_MODELVIEW);
			glPopMatrix();
			glMatrixMode(GL_PROJECTION);
			glPopMatrix();

			glMatrixMode(GL_TEXTURE);
			glPopMatrix();
		}
		else {
			object_error(&ob, "falied to create submit FBO");
			return false;
		}
		
		// tidy up:
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
		glPopAttrib();
		glPopClientAttrib();
		glMatrixMode(previousMatrixMode);
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, previousFBO);		
		
		return true;
	}
	
	bool fbo_check() {
		GLenum status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
		if (status != GL_FRAMEBUFFER_COMPLETE_EXT) {
			if (status == GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT) {
				object_error(&ob, "failed to create render to texture target GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT");
			}
			else if (status == GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT) {
				object_error(&ob, "failed to create render to texture target GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS");
			}
			else if (status == GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT) {
				object_error(&ob, "failed to create render to texture target GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT");
			}
			else if (status == GL_FRAMEBUFFER_UNSUPPORTED_EXT) {
				object_error(&ob, "failed to create render to texture target GL_FRAMEBUFFER_UNSUPPORTED");
			}
			else {
				object_error(&ob, "failed to create render to texture target %d", status);
			}
			return false;
		}
		return true;
	}
	
	// TODO videocamera
	// TODO battery, vibrate, render models

	//////////////////////////////////////////////////

	static void oculusrift_quit() {
		if (oculus_initialized) ovr_Shutdown();
		oculus_initialized = 0;
	}

	int oculusrift_init() {
		if (oculus_initialized) return 1;

		// init OVR SDK
		ovrInitParams initParams = { ovrInit_RequestVersion, OVR_MINOR_VERSION, NULL, 0, 0 };
		ovrResult result = ovr_Initialize(&initParams);
		if (OVR_FAILURE(result)) {
			object_error(0, "LibOVR: failed to initialize library");
			// if only this worked:
			//ovrErrorInfo errorInfo;
			//ovr_GetLastErrorInfo(&errorInfo);
			//object_error(0, "ovr_Initialize failed: %s", errorInfo.ErrorString);

			switch (result) {
			case ovrError_Initialize: object_error(&ob, "Generic initialization error."); break;
			case ovrError_LibLoad: object_error(&ob, "Couldn't load LibOVRRT."); break;
			case ovrError_LibVersion: object_error(&ob, "LibOVRRT version incompatibility."); break;
			case ovrError_ServiceConnection: object_error(&ob, "Couldn't connect to the OVR Service."); break;
			case ovrError_ServiceVersion: object_error(&ob, "OVR Service version incompatibility."); break;
			case ovrError_IncompatibleOS: object_error(&ob, "The operating system version is incompatible."); break;
			case ovrError_DisplayInit: object_error(&ob, "Unable to initialize the HMD display."); break;
			case ovrError_ServerStart:  object_error(&ob, "Unable to start the server. Is it already running?"); break;
			case ovrError_Reinitialization: object_error(&ob, "Attempted to re-initialize with a different version."); break;
			default: object_error(&ob, "unknown initialization error."); break;
			}
			oculus_initialized = 0;
		}
		else {

			quittask_install((method)oculusrift_quit, NULL);

			ovr_IdentifyClient("EngineName: Max/MSP/Jitter\n"
				"EngineVersion: 7\n"
				"EnginePluginName: [oculusrift]\n"
				"EngineEditor: true");
			oculus_initialized = 1;
		}
		return oculus_initialized;
	}

	bool oculus_connect() {
		ovrResult result = ovr_Create(&oculus.session, &oculus.luid);
		if (OVR_FAILURE(result)) {
			ovrErrorInfo errInfo;
			ovr_GetLastErrorInfo(&errInfo);
			object_error(&ob, "failed to create session: %s", errInfo.ErrorString);

			object_error(NULL, errInfo.ErrorString);
			return false;
		}

		object_post(&ob, "LibOVR SDK %s, runtime %s", OVR_VERSION_STRING, ovr_GetVersionString());
		
		// update our session status
		ovr_GetSessionStatus(oculus.session, &oculus.status);

		return true;
	}

	void oculus_disconnect() {
		if (oculus.session) {

			release_gpu_resources();

			ovr_Destroy(oculus.session);
			oculus.session = 0;
		}
	}
};

void vr_connect(Vr * x) { x->connect(); }
void vr_disconnect(Vr * x) { x->disconnect(); }
void vr_configure(Vr * x) { x->configure(); }
void vr_dest_changed(Vr * x) { x->dest_changed(); }
void vr_dest_closing(Vr * x) { x->dest_closing(); }
void vr_bang(Vr * x) { x->bang(); }

void vr_jit_gl_texture(Vr * x, t_symbol * s, long argc, t_atom * argv) {
	if (argc > 0 && atom_gettype(argv) == A_SYM) {
		x->jit_gl_texture(atom_getsym(argv));
	}
}

/*

void oculusrift_perf(oculusrift * x) {
x->perf();
}

void oculusrift_recenter(oculusrift * x) {
if (x->session) ovr_RecenterTrackingOrigin(x->session);
}*/

t_max_err vr_near_clip_set(Vr *x, t_object *attr, long argc, t_atom *argv) {
	x->near_clip = atom_getfloat(argv);
	x->configure();
	return 0;
}

t_max_err vr_far_clip_set(Vr *x, t_object *attr, long argc, t_atom *argv) {
	x->far_clip = atom_getfloat(argv);
	x->configure();
	return 0;
}

/*

t_max_err oculusrift_pixel_density_set(oculusrift *x, t_object *attr, long argc, t_atom *argv) {
x->pixel_density = atom_getfloat(argv);

x->configure();
return 0;
}

t_max_err oculusrift_max_fov_set(oculusrift *x, t_object *attr, long argc, t_atom *argv) {
x->max_fov = atom_getlong(argv);

x->configure();
return 0;
}

t_max_err oculusrift_tracking_level_set(oculusrift *x, t_object *attr, long argc, t_atom *argv) {
x->tracking_level = atom_getlong(argv);

x->configure();
return 0;
}
*/

void* vr_new(t_symbol* name, long argc, t_atom* argv) {
	Vr * x = (Vr *)object_alloc(this_class);
	if (x) {
		t_symbol * drawto = atom_getsym(argv);
		x = new (x)Vr(drawto);
		// apply attrs:
		attr_args_process(x, (short)argc, argv);
	}
	return x;
}


void vr_free(Vr* x) {
	x->~Vr();
}


void vr_assist(Vr* self, void* unused, t_assist_function m, long a, char* s) {
	if (m == ASSIST_INLET) { // inlet
		sprintf(s, "bang to update tracking, texture to submit, other messages");
	}
	else {	// outlet
		switch (a) {
		case 0: sprintf(s, "output/mirror texture"); break;
		case 1: sprintf(s, "to left eye camera"); break;
		case 2: sprintf(s, "to right eye camera"); break;
		case 3: sprintf(s, "to scene node (set texture dim)"); break;
		case 4: sprintf(s, "tracking state"); break;
		case 5: sprintf(s, "left controller"); break;
		case 6: sprintf(s, "right controller"); break;
		default: sprintf(s, "other messages"); break;
			//default: sprintf(s, "I am outlet %ld", a); break;
		}
	}
}




void ext_main(void* r) {

	ps_jit_gl_texture = gensym("jit_gl_texture");
	ps_glid = gensym("glid");
	ps_viewport = gensym("viewport");

	this_class = class_new("vr", (method)vr_new, (method)vr_free, sizeof(Vr), 0L, A_GIMME, 0);
	
	long ob3d_flags = jit_ob3d_flags::NO_MATRIXOUTPUT 
					//| jit_ob3d_flags::DOES_UI
					//| jit_ob3d_flags::NO_ROTATION_SCALE
					| jit_ob3d_flags::NO_POLY_VARS
					| jit_ob3d_flags::NO_BLEND
					| jit_ob3d_flags::NO_TEXTURE
					| jit_ob3d_flags::NO_MATRIXOUTPUT
					| jit_ob3d_flags::AUTO_ONLY
					| jit_ob3d_flags::NO_DEPTH
					| jit_ob3d_flags::NO_ANTIALIAS
					| jit_ob3d_flags::NO_FOG
					| jit_ob3d_flags::NO_LIGHTING_MATERIAL
					| jit_ob3d_flags::NO_SHADER
					| jit_ob3d_flags::NO_BOUNDS
					| jit_ob3d_flags::NO_COLOR
					;
	void * ob3d = jit_ob3d_setup(this_class, calcoffset(Vr, ob3d), ob3d_flags);
	
	// define our OB3D draw methods
	//jit_class_addmethod(this_class, (method)(vr_draw), "ob3d_draw", A_CANT, 0L);
	jit_class_addmethod(this_class, (method)(vr_dest_closing), "dest_closing", A_CANT, 0L);
	jit_class_addmethod(this_class, (method)(vr_dest_changed), "dest_changed", A_CANT, 0L);
	// must register for ob3d use
	jit_class_addmethod(this_class, (method)jit_object_register, "register", A_CANT, 0L);

	class_addmethod(this_class, (method)vr_assist,"assist",A_CANT,0);
		
 	class_addmethod(this_class, (method)vr_connect, "connect", 0);
 	class_addmethod(this_class, (method)vr_disconnect, "disconnect", 0);
 	class_addmethod(this_class, (method)vr_configure, "configure", 0);
	class_addmethod(this_class, (method)vr_bang, "bang", 0);
 	class_addmethod(this_class, (method)vr_jit_gl_texture, "jit_gl_texture", A_GIMME, 0);

	// oculus only?

	//class_addmethod(c, (method)oculusrift_perf, "perf", 0);
	//class_addmethod(c, (method)oculusrift_recenter, "recenter", 0);
	//CLASS_ATTR_FLOAT(c, "pixel_density", 0, oculusrift, pixel_density);
	//CLASS_ATTR_ACCESSORS(c, "pixel_density", NULL, oculusrift_pixel_density_set);

	// TODO: why is Rift not using max FOV (seems like the black overlay is not being made bigger - oculus bug?)
	//CLASS_ATTR_LONG(c, "max_fov", 0, oculusrift, max_fov);
	//CLASS_ATTR_ACCESSORS(c, "max_fov", NULL, oculusrift_max_fov_set);
	//CLASS_ATTR_STYLE_LABEL(c, "max_fov", 0, "onoff", "use maximum field of view");

	//CLASS_ATTR_LONG(c, "mirror", 0, oculusrift, mirror);
	//CLASS_ATTR_STYLE_LABEL(c, "mirror", 0, "onoff", "mirror oculus display in main window");

	//CLASS_ATTR_LONG(c, "tracking_level", 0, oculusrift, tracking_level);
	//CLASS_ATTR_ACCESSORS(c, "tracking_level", NULL, oculusrift_tracking_level_set);
	
	// vive-specific:
	//class_addmethod(this_class, (method)vr_vibrate, "vibrate", A_GIMME, 0);
	//class_addmethod(this_class, (method)vr_battery, "battery", A_GIMME, 0);
	
	CLASS_ATTR_FLOAT(this_class, "near_clip", 0, Vr, near_clip);
	CLASS_ATTR_ACCESSORS(this_class, "near_clip", NULL, vr_near_clip_set);
	CLASS_ATTR_FLOAT(this_class, "far_clip", 0, Vr, far_clip);
	CLASS_ATTR_ACCESSORS(this_class, "far_clip", NULL, vr_far_clip_set);
	
	class_register(CLASS_BOX, this_class);
}
