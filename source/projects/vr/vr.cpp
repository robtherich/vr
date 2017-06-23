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


#define VR_DEBUG_POST(fmt, ...) do {} while (0)
#ifdef WIN_VERSION
	// needed this for glFrameBuffer / GL_FRAMEBUFFER symbols
	// (and needed to comment out the jit.common.h include)
	#include "jit.gl.procs.h"
	#include "jit.gl.support.h"
	
	#define USE_DRIVERS 1

	//#define VR_DEBUG_POST(fmt, ...) do { object_post(0, "debug %s:%d:%s(): " fmt, __FILE__, __LINE__, __func__, __VA_ARGS__); } while (0)
	//#define VR_DEBUG_POST(fmt, ...) do { object_post(0, "debug line %d:%s(): " fmt, __LINE__, __func__, __VA_ARGS__); } while (0)
#else



#endif

	// needed to declare these here, as they aren't declared in c74_jitter.h:
	void * jit_object_findregistered(t_symbol *s);
	void * jit_object_register(void *x, t_symbol *s);
	void * jit_object_findregistered(c74::max::t_symbol *s);
	void * jit_object_register(void *x, c74::max::t_symbol *s);
	long jit_atom_arg_getsym(t_symbol ** c, long idx, long ac, t_atom *av);
	long jit_gl_report_error(char * prefix);

}

// Oculus:
#include "OVR_CAPI.h"
#include "OVR_CAPI_GL.h"
// steam:
// The OpenVR SDK:
#include "openvr.h"

#include "al_math.h"

static bool oculus_initialized = 0;

static t_symbol * ps_glid;
static t_symbol * ps_jit_gl_texture;
static t_symbol * ps_viewport;
static t_symbol * ps_frustum;
static t_symbol * ps_tracked_position;
static t_symbol * ps_tracked_quat;

static t_symbol * ps_head;
static t_symbol * ps_left_hand;
static t_symbol * ps_right_hand;

static t_symbol * ps_velocity;
static t_symbol * ps_angular_velocity;
static t_symbol * ps_trigger;
static t_symbol * ps_hand_trigger;
static t_symbol * ps_pad;
static t_symbol * ps_buttons;

static t_symbol * ps_oculus;
static t_symbol * ps_steam;

// jitter uses xyzw format
// glm constructor uses wxyz format
// though the data is stored in xyzw
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

glm::quat to_glm(ovrQuatf const q) {
	return glm::quat(q.w, q.x, q.y, q.z);
}

glm::vec3 to_glm(ovrVector3f const v) {
	return glm::vec3(v.x, v.y, v.z);
}

glm::vec3 to_glm(vr::HmdVector3_t const v) {
	return glm::vec3(v.v[0], v.v[1], v.v[2]);
}

glm::mat4 to_glm(vr::HmdMatrix34_t const m) {
	return glm::mat4(
		m.m[0][0], m.m[1][0], m.m[2][0], 0.0,
		m.m[0][1], m.m[1][1], m.m[2][1], 0.0,
		m.m[0][2], m.m[1][2], m.m[2][2], 0.0,
		m.m[0][3], m.m[1][3], m.m[2][3], 1.0f);
}

glm::mat4 to_glm(vr::HmdMatrix44_t const m) {
	return glm::mat4(
		m.m[0][0], m.m[1][0], m.m[2][0], m.m[3][0],
		m.m[0][1], m.m[1][1], m.m[2][1], m.m[3][1],
		m.m[0][2], m.m[1][2], m.m[2][2], m.m[3][2],
		m.m[0][3], m.m[1][3], m.m[2][3], m.m[3][3]);
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
	void * outlet_node;
	int attrs_ready = 0;
	int dest_ready = 0;
	
	// attrs:
	float near_clip = 0.15f;
	float far_clip = 100.f;
	t_atom_long preferred_driver_only = 0;
	t_atom_long glfinishhack = 0;
	t_symbol * driver;
	t_atom_long connected = 0;
	t_atom_long oculus_available = 0, steam_available = 0;

	glm::vec3 view_position;
	glm::quat view_quat;
	glm::mat4 view_mat; // aka modelview_mat
	glm::mat4 eye_mat[2]; // pose of each eye, in tracking space

	// guts:
	
	// FBO & texture that the scene is copied into
	// (we can't submit the jit_gl_texture directly)
	GLuint fbo_id = 0;
	GLuint rbo_id = 0;
	t_atom_long fbo_dim[2];

	// driver-specific:
	struct {
		ovrSession session = 0;
		ovrSessionStatus status;
		ovrHmdDesc hmd;
		ovrGraphicsLuid luid;
		ovrEyeRenderDesc eyeRenderDesc[2];
		ovrVector3f      hmdToEyeViewOffset[2];
		ovrLayerEyeFov layer;
		//ovrSizei pTextureDim;
		ovrTextureSwapChain textureChain = 0;
		ovrMirrorTexture mirrorTexture;
		long long frameIndex = 0;
		double sensorSampleTime = 0.f;    // sensorSampleTime is fed into the layer later
		int max_fov = 0; // use default field of view; set to 1 for maximum field of view
		float pixel_density = 1.f;
		int tracking_level = (int)ovrTrackingOrigin_FloorLevel;
	} oculus;

	struct {
		vr::IVRSystem *	hmd;
		t_symbol * driver;
		t_symbol * display;
		GLuint fbo_texture_id = 0;

		vr::TrackedDevicePose_t pRenderPoseArray[vr::k_unMaxTrackedDeviceCount];
		int mHandControllerDeviceIndex[2];
		glm::mat4 head2eye_mat[2];
		glm::mat4 m_mat4projectionEye[2];

		vr::IVRRenderModels * mRenderModels = 0;

		int use_camera = 0;
		vr::IVRTrackedCamera * mCamera = 0;
		vr::TrackedCameraHandle_t m_hTrackedCamera = INVALID_TRACKED_CAMERA_HANDLE;
		uint32_t	m_nCameraFrameWidth;
		uint32_t	m_nCameraFrameHeight;
		uint32_t	m_nCameraFrameBufferSize;
		uint32_t	m_nLastFrameSequence = 0;
		uint8_t		* m_pCameraFrameBuffer = 0;
		vr::EVRTrackedCameraFrameType frametype = vr::VRTrackedCameraFrameType_Undistorted;

	} steam;
	
	Vr(t_symbol * drawto) {
		// init Max object:
		jit_ob3d_new(this, drawto);
		// outlets create in reverse order:
		outlet_msg = outlet_new(&ob, NULL);
		//outlet_video = outlet_new(&ob, "jit_gl_texture");
		outlet_tracking = outlet_new(&ob, NULL);
		//outlet_node = outlet_new(&ob, NULL);
		outlet_eye[1] = outlet_new(&ob, NULL);
		outlet_eye[0] = outlet_new(&ob, NULL);
		outlet_node = outlet_new(&ob, NULL);

		driver = gensym("oculus");
		
		// some whatever defaults, will get overwritten when driver connects
		fbo_dim[0] = 1920;
		fbo_dim[1] = 1080;

		// default eye positions (for offline testing)
		for (int eye = 0; eye < 2; eye++) {
			float ipd = 0.61; // an average adult
			float eye_height = 1.59;
			float eye_forward = 0.095; // a typical distance from center of head to eye plane
			glm::vec3 p(eye ? ipd / 2.f : -ipd / 2.f, eye_height, -eye_forward);
			eye_mat[eye] = glm::translate(glm::mat4(1.0f), p);

			steam.mHandControllerDeviceIndex[eye] = -1;
		}

		update_availability();
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

	// Jitter GL context changed, need to reallocate GPU objects
	// happens on construction (if context already existed)
	// or when gl context is created
	// e.g. when creating jit.world or entering/leaving fullscreen
	t_jit_err dest_changed() {
		VR_DEBUG_POST("dest_changed");
		// mark drawto gpu context as usable
		// might not allocate gpu resources immediately, depends on whether driver is also connected
		dest_ready = 1;
		object_attr_touch(&ob, gensym("dest_ready"));

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
		VR_DEBUG_POST("dest_closing");

		// automatically disconnect at this point
		// since without a Jitter GPU context, there's nothing we can do
		// and disconnecting will free up the driver for other connections
		// e.g. as needed when entering/leaving fullscreen
		disconnect();

		// mark drawto gpu context as unusable
		dest_ready = 0;
		object_attr_touch(&ob, gensym("dest_ready"));

		release_gpu_resources();

		return JIT_ERR_NONE;
	}

	void update_availability() {
#ifdef USE_DRIVERS
		oculus_available = oculus_is_available();
		object_attr_touch(&ob, gensym("oculus_available"));
		steam_available = steam_is_available();
		object_attr_touch(&ob, gensym("steam_available"));
#endif
		t_atom a[1];

		atom_setlong(a, oculus_available);
		outlet_anything(outlet_msg, gensym("oculus_available"), 1, a);
		atom_setlong(a, steam_available);
		outlet_anything(outlet_msg, gensym("steam_available"), 1, a);
	}

	// attempt to acquire the HMD
	bool connect() {
		if (connected) return true; // because we're already connected!
		VR_DEBUG_POST("connect");

		update_availability();

		#ifdef USE_DRIVERS
		// figure out which driver we want to use:
		if (driver == ps_steam) {
			connected = steam_connect();
			if (!connected && !preferred_driver_only) {
				connected = oculus_connect();
			}
		} else {
			connected = oculus_connect();
			if (!connected && !preferred_driver_only) {
				connected = steam_connect();
			}
		}
		#endif

		object_attr_touch(&ob, gensym("connected"));
		object_attr_touch(&ob, gensym("driver"));


		VR_DEBUG_POST("connected %i", connected);

		// whether or not we connected, configure
		// so that offline simulation still works
		configure();


		t_atom a[2];
		atom_setlong(a, connected);
		outlet_anything(outlet_msg, gensym("connected"), 1, a);
		atom_setsym(a, driver);
		outlet_anything(outlet_msg, gensym("driver"), 1, a);
		atom_setlong(a + 0, fbo_dim[0]);
		atom_setlong(a + 1, fbo_dim[1]);
		outlet_anything(outlet_msg, _jit_sym_dim, 2, a);

		// if connected and gpu is ready, go ahead & make what we need
		if (connected && dest_ready) {
			create_gpu_resources();
		}
		return connected;
	}
	
	// release the HMD
	void disconnect() {
		if (!connected) return;
		VR_DEBUG_POST("disconnect");

		// TODO: driver-specific stuff
		#ifdef USE_DRIVERS
		if (driver == ps_steam) {
			steam_disconnect();
		}
		else {
			oculus_disconnect();
		}
		#endif
		
		connected = 0;
		object_attr_touch(&ob, gensym("connected"));
	}

	// called whenever HMD properties change
	// will dump a lot of information to the last outlet
	void configure() {
		VR_DEBUG_POST("configure %d", connected);
		t_atom a[6];
		
		if (connected) {
			// TODO get driver details & output
			#ifdef USE_DRIVERS
			if (driver == ps_steam) {
				steam_configure();
			}
			else {
				oculus_configure();
			}
			#endif
		}

		// output recommended texture dim:
		atom_setlong(a + 0, fbo_dim[0]);
		atom_setlong(a + 1, fbo_dim[1]);
		outlet_anything(outlet_node, _jit_sym_dim, 2, a);
		atom_setlong(a, 0);
		outlet_anything(outlet_node, _jit_sym_adapt, 1, a);

		// output camera properties:
		atom_setsym(a, ps_frustum);
		outlet_anything(outlet_eye[0], gensym("projection_mode"), 1, a);
		outlet_anything(outlet_eye[1], gensym("projection_mode"), 1, a);
		
		// viewports:
		atom_setfloat(a + 0, 0.);
		atom_setfloat(a + 1, 0.);
		atom_setfloat(a + 2, 0.5);
		atom_setfloat(a + 3, 1.);
		outlet_anything(outlet_eye[0], ps_viewport, 4, a);
		atom_setfloat(a + 0, 0.5);
		outlet_anything(outlet_eye[1], ps_viewport, 4, a);
			
	}
	

	void create_gpu_resources() {
		if (!connected && !dest_ready) return; // we're not ready yet
		if (fbo_id) return; // we already did it

		VR_DEBUG_POST("create_gpu_resources");

		// get drawto context:
		t_symbol *context = object_attr_getsym(this, gensym("drawto"));

		// create the FBO used to pass the scene texture to the driver:
		if (!fbo_id) {
			glGenFramebuffersEXT(1, &fbo_id);
		}

		#ifdef USE_DRIVERS
		if (driver == ps_steam) {
			steam_create_gpu_resources();
		}
		else {
			oculus_create_gpu_resources();
		}
		#endif
	}

	void release_gpu_resources() {
		
		// release associated resources:
		if (fbo_id) {
			VR_DEBUG_POST("release_gpu_resources");
			glDeleteFramebuffersEXT(1, &fbo_id);
			fbo_id = 0;

			// TODO driver specific
			#ifdef USE_DRIVERS
			if (driver == ps_steam) {
				steam_release_gpu_resources();
			}
			else {
				oculus_release_gpu_resources();
			}
			#endif
		}
	}
	
	// poll HMD for events
	// most importantly, it picks up the current HMD pose
	// this should be the *last* thing to happen before rendering the scene
	// nothing time-consuming should happen between bang() and render
	void bang() {
		
		// get desired view matrix (from @position and @quat attrs)
		object_attr_getfloat_array(this, _jit_sym_position, 3, &view_position.x);
		object_attr_getfloat_array(this, _jit_sym_quat, 4, &view_quat.x);
		view_mat = glm::translate(glm::mat4(1.0f), view_position) * mat4_cast(view_quat);
		
		// TODO: video (or separate message for this?)
		
		// TODO: driver poll events
		if (connected) {
			#ifdef USE_DRIVERS
			if (driver == ps_steam) {
				steam_bang();
			}
			else {
				oculus_bang();
			}
			#endif
		}
		else {
			// perhaps, poll for availability?
		}

		// always output camera poses here (so it works even if not currently tracking)
		t_atom a[4];
		for (int eye = 0; eye < 2; eye++) {
			glm::mat4 world_mat = view_mat * eye_mat[eye];

			glm::vec3 p = glm::vec3(world_mat[3]); // the translation component
			atom_setfloat(a + 0, p.x);
			atom_setfloat(a + 1, p.y);
			atom_setfloat(a + 2, p.z);
			outlet_anything(outlet_eye[eye], _jit_sym_position, 3, a);

			glm::quat q = glm::quat_cast(world_mat); // the orientation component
			atom_setfloat(a + 0, q.x);
			atom_setfloat(a + 1, q.y);
			atom_setfloat(a + 2, q.z);
			atom_setfloat(a + 3, q.w);
			outlet_anything(outlet_eye[eye], _jit_sym_quat, 4, a);
		}
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

		// get input properties:
		GLuint input_texture_id = object_attr_getlong(jit_texture, ps_glid);
		t_atom_long input_texture_dim[2];
		object_attr_getlong_array(jit_texture, _jit_sym_dim, 2, input_texture_dim);

		if (connected) {
			// submit it to the driver
			if (!fbo_id) {
				// TODO try to allocate FBO for copying Jitter texture to driver?
				create_gpu_resources();
				if (!fbo_id) {
					// just bug out at this point?
					object_error(&ob, "no fbo yet");
					return;	// no texture to copy from.
				}
			}

			// TODO driver specific
			#ifdef USE_DRIVERS
			if (driver == ps_steam) {
				if (!steam_submit_texture(input_texture_id, input_texture_dim)) {
					object_error(&ob, "problem submitting texture");
				}
			}
			else {
				if (!oculus_submit_texture(input_texture_id, input_texture_dim)) {
					object_error(&ob, "problem submitting texture");
				}
			}
			#endif
		}
		
		t_atom a[1];
		if (connected) {
			// TODO: output mirror textures here if desired?
		}
	}
	
	//////////////////////////////////////////////////////////////////////////////////////
	
	bool fbo_copy_texture(GLuint 		input_texture_id, 
							 t_atom_long 	input_texture_dim[2],
							 GLuint 		fbo_id, 
							 GLuint 		fbo_texture_id, 
							 t_atom_long 	fbo_dim[2],
							 bool flipY = true) {
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

			glViewport(0, 0, fbo_dim[0], fbo_dim[1]);
			
			glMatrixMode(GL_PROJECTION);
			glPushMatrix();
			glLoadIdentity();
			if (flipY) {
				glOrtho(0.0, fbo_dim[0], 0.0, fbo_dim[1], -1, 1);
			}
			else {
				glOrtho(0.0, fbo_dim[0], fbo_dim[1], 0., -1, 1);
			}

			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glLoadIdentity();

			glClearColor(0, 0, 0, 1);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			glActiveTexture(GL_TEXTURE0);
			glClientActiveTexture(GL_TEXTURE0);
			glEnable(GL_TEXTURE_RECTANGLE_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB, input_texture_id);
			
			// do not need blending if we use black border for alpha and replace env mode, saves a buffer wipe
			// we can do this since our image draws over the complete surface of the FBO, no pixel goes untouched.

			glDisable(GL_BLEND);
			glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

			// move to VA for rendering
			// TODO flip vertical?
			GLfloat tex_coords[] = {
				(GLfloat)input_texture_dim[0], 0.f,
				0.0, 0.f,
				0.0, (GLfloat)input_texture_dim[1],
				(GLfloat)input_texture_dim[0], (GLfloat)input_texture_dim[1]
			};

			GLfloat verts[] = {
				(GLfloat)fbo_dim[0], (GLfloat)fbo_dim[1],
				0.0, (GLfloat)fbo_dim[1],
				0.0, 0.0,
				(GLfloat)fbo_dim[0], 0.0
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
	
#ifdef USE_DRIVERS

	static void oculusrift_quit() {
		if (oculus_initialized) ovr_Shutdown();
		oculus_initialized = 0;
	}

	bool oculus_is_available() {
		ovrDetectResult res = ovr_Detect(250); // ms timeout
		return (res.IsOculusServiceRunning && res.IsOculusHMDConnected);
	}

	int oculusrift_init() {
		if (!oculus_available) return 0;

		if (oculus_initialized) return 1;
		VR_DEBUG_POST("oculus init");

		// init OVR SDK
		ovrInitParams initParams = { ovrInit_RequestVersion, OVR_MINOR_VERSION, NULL, 0, 0 };
		ovrResult result = ovr_Initialize(&initParams);
		if (OVR_FAILURE(result)) {
			oculus_initialized = 0;

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
		}
		else {

			quittask_install((method)oculusrift_quit, NULL);

			ovr_IdentifyClient("EngineName: Max/MSP/Jitter\n"
				"EngineVersion: 7\n"
				"EnginePluginName: [vr]\n"
				"EngineEditor: true");
			oculus_initialized = 1;


			VR_DEBUG_POST("oculus initialized OK");
		}
		return oculus_initialized;
	}

	bool oculus_connect() {
		if (!oculusrift_init()) return false;

		VR_DEBUG_POST("oculus connect");

		ovrResult result = ovr_Create(&oculus.session, &oculus.luid);
		if (OVR_FAILURE(result)) {
			ovrErrorInfo errInfo;
			ovr_GetLastErrorInfo(&errInfo);
			object_error(&ob, "failed to create session: %s", errInfo.ErrorString);

			object_error(NULL, errInfo.ErrorString);
			return false;
		}

		driver = ps_oculus;
		return true;
	}

	void oculus_disconnect() {
		if (oculus.session) {
			VR_DEBUG_POST("oculus disconnect");

			release_gpu_resources();

			ovr_Destroy(oculus.session);
			oculus.session = 0;

			// let go of driver, so steam can use it
			oculusrift_quit();
		}
	}

	void oculus_configure() {
		VR_DEBUG_POST("oculus configure");
		if (!oculus.session) {
			object_error(&ob, "no Oculus session to configure");
			return;
		}

		t_atom a[2];

		// maybe never: support disabling tracking options via ovr_ConfigureTracking()

		oculus.hmd = ovr_GetHmdDesc(oculus.session);
		// Use hmd members and ovr_GetFovTextureSize() to determine graphics configuration


		atom_setsym(a, gensym(OVR_VERSION_STRING));
		outlet_anything(outlet_msg, gensym("SDK"), 1, a);
		atom_setsym(a, gensym(ovr_GetVersionString()));
		outlet_anything(outlet_msg, gensym("runtime"), 1, a);

		// TODO complete list of useful info from https://developer.oculus.com/documentation/pcsdk/latest/concepts/dg-sensor/
#define HMD_CASE(T) case T: { \
            atom_setsym(a, gensym( #T )); \
            outlet_anything(outlet_msg, gensym("hmdType"), 1, a); \
            break; \
			        }
		switch (oculus.hmd.Type) {
			HMD_CASE(ovrHmd_CV1)
				HMD_CASE(ovrHmd_DK1)
				HMD_CASE(ovrHmd_DKHD)
				HMD_CASE(ovrHmd_DK2)
		default: {
				atom_setsym(a, gensym("unknown"));
				outlet_anything(outlet_msg, gensym("Type"), 1, a);
			}
		}
#undef HMD_CASE
		atom_setsym(a, gensym(oculus.hmd.SerialNumber));
		outlet_anything(outlet_msg, gensym("serial"), 1, a);
		atom_setsym(a, gensym(oculus.hmd.Manufacturer));
		outlet_anything(outlet_msg, gensym("Manufacturer"), 1, a);
		atom_setsym(a, gensym(oculus.hmd.ProductName));
		outlet_anything(outlet_msg, gensym("ProductName"), 1, a);
		atom_setlong(a, (oculus.hmd.VendorId));
		outlet_anything(outlet_msg, gensym("VendorId"), 1, a);
		atom_setlong(a, (oculus.hmd.ProductId));
		outlet_anything(outlet_msg, gensym("ProductId"), 1, a);
		atom_setlong(a, (oculus.hmd.AvailableHmdCaps));
		outlet_anything(outlet_msg, gensym("AvailableHmdCaps"), 1, a);
		atom_setlong(a, (oculus.hmd.DefaultHmdCaps));
		outlet_anything(outlet_msg, gensym("DefaultHmdCaps"), 1, a);
		atom_setlong(a, (oculus.hmd.AvailableTrackingCaps));
		outlet_anything(outlet_msg, gensym("AvailableTrackingCaps"), 1, a);
		atom_setlong(a, (oculus.hmd.DefaultTrackingCaps));
		outlet_anything(outlet_msg, gensym("DefaultTrackingCaps"), 1, a);
		atom_setfloat(a, (oculus.hmd.DisplayRefreshRate));
		outlet_anything(outlet_msg, gensym("DisplayRefreshRate"), 1, a);
		atom_setlong(a, oculus.hmd.FirmwareMajor);
		atom_setlong(a + 1, oculus.hmd.FirmwareMinor);
		outlet_anything(outlet_msg, gensym("Firmware"), 2, a);
		ovrSizei resolution = oculus.hmd.Resolution;
		atom_setlong(a + 0, resolution.w);
		atom_setlong(a + 1, resolution.h);
		outlet_anything(outlet_msg, gensym("resolution"), 2, a);


		ovrSizei recommenedTex0Size, recommenedTex1Size;
		//MaxEyeFov - Maximum optical field of view that can be practically rendered for each eye.
		if (oculus.max_fov) {
			recommenedTex0Size = ovr_GetFovTextureSize(oculus.session, ovrEye_Left, oculus.hmd.MaxEyeFov[0], oculus.pixel_density);
			recommenedTex1Size = ovr_GetFovTextureSize(oculus.session, ovrEye_Right, oculus.hmd.MaxEyeFov[1], oculus.pixel_density);
		}
		else {
			recommenedTex0Size = ovr_GetFovTextureSize(oculus.session, ovrEye_Left, oculus.hmd.DefaultEyeFov[0], oculus.pixel_density);
			recommenedTex1Size = ovr_GetFovTextureSize(oculus.session, ovrEye_Right, oculus.hmd.DefaultEyeFov[1], oculus.pixel_density);
		}
		// Initialize our single full screen Fov layer.
		// (needs to happen after textureset_create)
		oculus.layer.Header.Type = ovrLayerType_EyeFov;
		oculus.layer.Header.Flags = 0;// ovrLayerFlag_TextureOriginAtBottomLeft;   // Because OpenGL. was 0.
		oculus.layer.Viewport[0].Pos.x = 0;
		oculus.layer.Viewport[0].Pos.y = 0;
		oculus.layer.Viewport[0].Size.w = recommenedTex0Size.w;
		oculus.layer.Viewport[0].Size.h = recommenedTex0Size.h;
		oculus.layer.Viewport[1].Pos.x = recommenedTex0Size.w;
		oculus.layer.Viewport[1].Pos.y = 0;
		oculus.layer.Viewport[1].Size.w = recommenedTex1Size.w;
		oculus.layer.Viewport[1].Size.h = recommenedTex1Size.h;

		// determine the recommended texture size for scene capture:
		fbo_dim[0] = recommenedTex0Size.w + recommenedTex1Size.w; // side-by-side
		fbo_dim[1] = AL_MAX(recommenedTex0Size.h, recommenedTex1Size.h);

		switch (oculus.tracking_level) {
		case int(ovrTrackingOrigin_FloorLevel) :
			// FloorLevel will give tracking poses where the floor height is 0
			// Tracking system origin reported at floor height.
			// Prefer using this origin when your application requires the physical floor height to match the virtual floor height, such as standing experiences. When used, all poses in ovrTrackingState are reported as an offset transform from the profile calibrated floor pose. Calling ovr_RecenterTrackingOrigin will recenter the X & Z axes as well as yaw, but the Y-axis (i.e. height) will continue to be reported using the floor height as the origin for all poses.
			ovr_SetTrackingOriginType(oculus.session, ovrTrackingOrigin_FloorLevel);
			break;
		default:
			// Tracking system origin reported at eye (HMD) height.
			// Prefer using this origin when your application requires matching user's current physical head pose to a virtual head pose without any regards to a the height of the floor. Cockpit-based, or 3rd-person experiences are ideal candidates. When used, all poses in ovrTrackingState are reported as an offset transform from the profile calibrated or recentered HMD pose. 
			ovr_SetTrackingOriginType(oculus.session, ovrTrackingOrigin_EyeLevel);
		};

		VR_DEBUG_POST("oculus configured");
	}

	bool oculus_create_gpu_resources() {
		if (!oculus.session) return false;
		if (!oculus.textureChain) {
			VR_DEBUG_POST("oculus create gpu");
			ovrTextureSwapChainDesc desc = {};
			desc.Type = ovrTexture_2D;
			desc.ArraySize = 1;
			desc.Width = fbo_dim[0];
			desc.Height = fbo_dim[1];
			desc.MipLevels = 1;
			desc.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;
			desc.SampleCount = 1;
			desc.StaticImage = ovrFalse;
			ovrResult result = ovr_CreateTextureSwapChainGL(oculus.session, &desc, &oculus.textureChain);
			if (result != ovrSuccess) {
				ovrErrorInfo errInfo;
				ovr_GetLastErrorInfo(&errInfo);
				object_error(&ob, "failed to create texture set: %s", errInfo.ErrorString);
				return false;
			}

			// unused?
			int length = 0;
			ovr_GetTextureSwapChainLength(oculus.session, oculus.textureChain, &length);

			// we can update the layer too here:
			oculus.layer.ColorTexture[0] = oculus.textureChain;
			oculus.layer.ColorTexture[1] = oculus.textureChain;

			VR_DEBUG_POST("oculus gpu resources created");

		}
		return true;
	}

	void oculus_release_gpu_resources() {
		VR_DEBUG_POST("oculus release gpu");
		if (oculus.session && oculus.textureChain) {
			ovr_DestroyTextureSwapChain(oculus.session, oculus.textureChain);
			oculus.textureChain = 0;
		}
	}

	void oculus_bang() {
		if (!oculus.session) return;

		ovrResult res = ovr_GetSessionStatus(oculus.session, &oculus.status);
		if (oculus.status.ShouldQuit) {
			// the HMD display will return to Oculus Home
			// don't want to quit, but at least notify patcher:
			outlet_anything(outlet_msg, gensym("quit"), 0, NULL);

			disconnect();
			return;
		}
		if (oculus.status.ShouldRecenter) {
			ovr_RecenterTrackingOrigin(oculus.session);
			/*
			Expose attr to defeat this?
			Some applications may have reason to ignore the request or to implement it
			via an internal mechanism other than via ovr_RecenterTrackingOrigin. In such
			cases the application can call ovr_ClearShouldRecenterFlag() to cause the
			recenter request to be cleared.
			*/
		}

		if (!oculus.status.HmdPresent) {
			// TODO: disconnect?
			return;
		}
		if (oculus.status.DisplayLost) {
			/*
			Destroy any TextureSwapChains or mirror textures.
			Call ovrDestroy.
			Poll ovrSessionStatus::HmdPresent until true.
			Call ovrCreate to recreate the session.
			Recreate any TextureSwapChains or mirror textures.
			Resume the application.
			ovrDetect() ??
			*/
		}

		// TODO: expose these as gettable attrs?
		//status.HmdMounted // true if the HMD is currently on the head
		// status.IsVisible // True if the game or experience has VR focus and is visible in the HMD.

		// TODO: does SDK provide notification of Rift being reconnected?

		t_atom a[6];

		// Call ovr_GetRenderDesc each frame to get the ovrEyeRenderDesc, as the returned values (e.g. HmdToEyeOffset) may change at runtime.
		if (oculus.max_fov) {
			oculus.eyeRenderDesc[0] = ovr_GetRenderDesc(oculus.session, ovrEye_Left, oculus.hmd.MaxEyeFov[0]);
			oculus.eyeRenderDesc[1] = ovr_GetRenderDesc(oculus.session, ovrEye_Right, oculus.hmd.MaxEyeFov[1]);
		} else {
			oculus.eyeRenderDesc[0] = ovr_GetRenderDesc(oculus.session, ovrEye_Left, oculus.hmd.DefaultEyeFov[0]);
			oculus.eyeRenderDesc[1] = ovr_GetRenderDesc(oculus.session, ovrEye_Right, oculus.hmd.DefaultEyeFov[1]);
		}
		oculus.hmdToEyeViewOffset[0] = oculus.eyeRenderDesc[0].HmdToEyeOffset;
		oculus.hmdToEyeViewOffset[1] = oculus.eyeRenderDesc[1].HmdToEyeOffset;

		// now tracking data:
		// Query the HMD for the predicted tracking state
		double displayMidpointSeconds = ovr_GetPredictedDisplayTime(oculus.session, oculus.frameIndex);
		ovrTrackingState ts = ovr_GetTrackingState(oculus.session, displayMidpointSeconds, ovrTrue);
		if (ts.StatusFlags & (ovrStatus_OrientationTracked | ovrStatus_PositionTracked)) {

			// Computes offset eye poses based on headPose returned by ovrTrackingState.
			// use the tracking state to update the layers (part of how timewarp works)
			ovr_CalcEyePoses(ts.HeadPose.ThePose, oculus.hmdToEyeViewOffset, oculus.layer.RenderPose);

			// update the camera poses/frusta accordingly
			for (int eye = 0; eye < 2; eye++) {

				// update the layer info too:
				oculus.layer.Fov[eye] = oculus.eyeRenderDesc[eye].Fov;
				oculus.layer.SensorSampleTime = oculus.sensorSampleTime;

				// get the tracking-space pose & convert to mat4
				const ovrPosef& pose = oculus.layer.RenderPose[eye];
				eye_mat[eye] = glm::translate(glm::mat4(1.0f), to_glm(pose.Position))
					* mat4_cast(to_glm(pose.Orientation));

				// TODO: proj matrix doesn't need to be calculated every frame; only when fov/near/far/layer data changes
				// projection
				const ovrFovPort& fov = oculus.layer.Fov[eye];
				atom_setfloat(a + 0, -fov.LeftTan * near_clip);
				atom_setfloat(a + 1, fov.RightTan * near_clip);
				atom_setfloat(a + 2, -fov.DownTan * near_clip);
				atom_setfloat(a + 3, fov.UpTan * near_clip);
				atom_setfloat(a + 4, near_clip);
				atom_setfloat(a + 5, far_clip);
				outlet_anything(outlet_eye[eye], ps_frustum, 6, a);
			}
		}

		{
			// Headset tracking data:
			{
				t_symbol * id = ps_head;
				
				// raw tracking data
				const ovrPosef& pose = ts.HeadPose.ThePose;

				glm::mat4 mat = glm::translate(glm::mat4(1.0f), to_glm(pose.Position))
					* mat4_cast(to_glm(pose.Orientation));
				glm::vec3 p = glm::vec3(mat[3]); // the translation component
				glm::quat q = glm::quat_cast(mat); // the orientation component

				// adjusted to world space
				glm::mat4 world_mat = view_mat * mat;
				glm::vec3 p1 = glm::vec3(world_mat[3]); // the translation component
				glm::quat q1 = glm::quat_cast(world_mat); // the orientation component

				atom_setsym(a + 0, ps_tracked_position);
				atom_setfloat(a + 1, p.x);
				atom_setfloat(a + 2, p.y);
				atom_setfloat(a + 3, p.z);
				outlet_anything(outlet_tracking, id, 4, a);

				atom_setsym(a + 0, ps_tracked_quat);
				atom_setfloat(a + 1, q.x);
				atom_setfloat(a + 2, q.y);
				atom_setfloat(a + 3, q.z);
				atom_setfloat(a + 4, q.w);
				outlet_anything(outlet_tracking, id, 5, a);

				atom_setsym(a + 0, _jit_sym_position);
				atom_setfloat(a + 1, p1.x);
				atom_setfloat(a + 2, p1.y);
				atom_setfloat(a + 3, p1.z);
				outlet_anything(outlet_tracking, id, 4, a);

				atom_setsym(a + 0, _jit_sym_quat);
				atom_setfloat(a + 1, q1.x);
				atom_setfloat(a + 2, q1.y);
				atom_setfloat(a + 3, q1.z);
				atom_setfloat(a + 4, q1.w);
				outlet_anything(outlet_tracking, id, 5, a);
			}

			// controllers:
			ovrInputState inputState;
			if (OVR_SUCCESS(ovr_GetInputState(oculus.session, ovrControllerType_Touch, &inputState))) {
				for (int i = 0; i < 2; i++) {

					t_symbol * id = i ? ps_right_hand : ps_left_hand;

					const ovrPosef& pose = ts.HandPoses[i].ThePose;

					glm::mat4 mat = glm::translate(glm::mat4(1.0f), to_glm(pose.Position))
						* mat4_cast(to_glm(pose.Orientation));
					glm::vec3 p = glm::vec3(mat[3]); // the translation component
					glm::quat q = glm::quat_cast(mat); // the orientation component
					
					// adjusted to world space
					glm::mat4 world_mat = view_mat * mat;
					glm::vec3 p1 = glm::vec3(world_mat[3]); // the translation component
					glm::quat q1 = glm::quat_cast(world_mat); // the orientation component

					atom_setsym(a + 0, ps_tracked_position);
					atom_setfloat(a + 1, p.x);
					atom_setfloat(a + 2, p.y);
					atom_setfloat(a + 3, p.z);
					outlet_anything(outlet_tracking, id, 4, a);

					atom_setsym(a + 0, ps_tracked_quat);
					atom_setfloat(a + 1, q.x);
					atom_setfloat(a + 2, q.y);
					atom_setfloat(a + 3, q.z);
					atom_setfloat(a + 4, q.w);
					outlet_anything(outlet_tracking, id, 5, a);

					atom_setsym(a + 0, _jit_sym_position);
					atom_setfloat(a + 1, p1.x);
					atom_setfloat(a + 2, p1.y);
					atom_setfloat(a + 3, p1.z);
					outlet_anything(outlet_tracking, id, 4, a);

					atom_setsym(a + 0, _jit_sym_quat);
					atom_setfloat(a + 1, q1.x);
					atom_setfloat(a + 2, q1.y);
					atom_setfloat(a + 3, q1.z);
					atom_setfloat(a + 4, q1.w);
					outlet_anything(outlet_tracking, id, 5, a);

					// velocities:
					// note that these are in tracking space
					glm::vec3 vel = to_glm(ts.HandPoses[i].LinearVelocity);
					glm::vec3 angvel = to_glm(ts.HandPoses[i].AngularVelocity);
					// rotated into world space (TODO is this appropriate? rotate or unrotate?)
					vel = quat_rotate(view_quat, vel);
					angvel = quat_rotate(view_quat, angvel);

					atom_setsym(a + 0, ps_velocity);
					atom_setfloat(a + 1, vel.x);
					atom_setfloat(a + 2, vel.y);
					atom_setfloat(a + 3, vel.z);
					outlet_anything(outlet_tracking, id, 4, a);

					atom_setsym(a + 0, ps_angular_velocity);
					atom_setfloat(a + 1, angvel.x);
					atom_setfloat(a + 2, angvel.y);
					atom_setfloat(a + 3, angvel.z);
					outlet_anything(outlet_tracking, id, 4, a);

					// buttons
					atom_setsym(a + 0, ps_trigger);
					atom_setlong(a + 1, inputState.IndexTrigger[i] > 0.25);
					atom_setfloat(a + 2, inputState.IndexTrigger[i]);
					outlet_anything(outlet_tracking, id, 3, a);

					atom_setsym(a + 0, ps_hand_trigger);
					atom_setlong(a + 1, inputState.HandTrigger[i] > 0.25);
					atom_setfloat(a + 2, inputState.HandTrigger[i]);
					outlet_anything(outlet_tracking, id, 3, a);

					atom_setsym(a + 0, ps_pad);
					atom_setlong(a + 1, inputState.Touches & (i ? ovrButton_RThumb : ovrButton_LThumb));
					atom_setfloat(a + 2, inputState.Thumbstick[i].x);
					atom_setfloat(a + 3, inputState.Thumbstick[i].y);
					atom_setlong(a + 4, inputState.Buttons & (i ? ovrButton_RThumb : ovrButton_LThumb));
					outlet_anything(outlet_tracking, id, 5, a);

					atom_setsym(a + 0, ps_buttons);
					atom_setlong(a + 1, inputState.Buttons & (i ? ovrButton_A : ovrButton_X));
					atom_setlong(a + 2, inputState.Buttons & (i ? ovrButton_B : ovrButton_Y));
					outlet_anything(outlet_tracking, id, 3, a);
				}
			}
		}
	}

	bool oculus_submit_texture(GLuint input_texture_id, t_atom_long input_texture_dim[2]) {
		if (!oculus.textureChain) {
			object_error(&ob, "no texture set yet");
			return false;
		}

		// get our next destination texture in the texture chain:
		int curIndex;
		ovr_GetTextureSwapChainCurrentIndex(oculus.session, oculus.textureChain, &curIndex);
		GLuint oculus_target_texture_id;
		ovr_GetTextureSwapChainBufferGL(oculus.session, oculus.textureChain, curIndex, &oculus_target_texture_id);

		// TODO: check success
		if (!fbo_copy_texture(input_texture_id, input_texture_dim, 
			fbo_id, oculus_target_texture_id, fbo_dim, true)) {
			object_error(&ob, "problem copying texture");
			return false;
		}

		// and commit it
		if (!OVR_SUCCESS(ovr_CommitTextureSwapChain(oculus.session, oculus.textureChain))) {
			object_error(&ob, "problem committing texture chain");
		}

		// Submit frame with one layer we have.
		// ovr_SubmitFrame returns once frame present is queued up and the next texture slot in the ovrSwatextureChain is available for the next frame. 
		ovrLayerHeader* layers = &oculus.layer.Header;
		ovrResult       result = ovr_SubmitFrame(oculus.session, oculus.frameIndex, nullptr, &layers, 1);
		if (result == ovrError_DisplayLost) {
			/*
			TODO: If you receive ovrError_DisplayLost, the device was removed and the session is invalid.
			Release the shared resources (ovr_DestroySwatextureChain), destroy the session (ovr_Destory),
			recreate it (ovr_Create), and create new resources (ovr_CreateSwatextureChainXXX).
			The application's existing private graphics resources do not need to be recreated unless
			the new ovr_Create call returns a different GraphicsLuid.
			*/
			object_error(&ob, "fatal error connection lost.");

			disconnect();

			return false;

		} else {
			oculus.frameIndex++;

			return true;
		}
	}
	

	/////////////////////////////////////////////////////////////////////////////////////////////////

	static t_symbol * steam_get_tracked_device_name(vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL)
	{
		uint32_t unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, NULL, 0, peError);
		if (unRequiredBufferLen == 0) return _jit_sym_nothing;

		char *pchBuffer = new char[unRequiredBufferLen];
		unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, pchBuffer, unRequiredBufferLen, peError);
		t_symbol * sResult = gensym(pchBuffer);
		delete[] pchBuffer;
		return sResult;
	}

	bool steam_is_available() {
		return (vr::VR_IsRuntimeInstalled() && vr::VR_IsHmdPresent());
	}

	bool steam_connect() {

		if (!steam_available) return false;

		vr::EVRInitError eError = vr::VRInitError_None;
		steam.hmd = vr::VR_Init(&eError, vr::VRApplication_Scene);
		if (eError != vr::VRInitError_None) {
			steam.hmd = 0;
			object_error(&ob, "Unable to init VR runtime: %s", vr::VR_GetVRInitErrorAsEnglishDescription(eError));
			return false;
		}
		if (!vr::VRCompositor()) {
			object_error(&ob, "Compositor initialization failed.");
			return false;
		}

		/*
		steam.mRenderModels = (vr::IVRRenderModels *)vr::VR_GetGenericInterface(vr::IVRRenderModels_Version, &eError);
		if (!steam.mRenderModels) {
			object_error(&ob, "Unable to init VR runtime: %s", vr::VR_GetVRInitErrorAsEnglishDescription(eError));
		}*/

		// TODO: move this out of connect() and into an attr setter? or only run if use_camera enabled?
		if (steam.use_camera) {
			steam.mCamera = vr::VRTrackedCamera(); // (vr::IVRTrackedCamera *)vr::VR_GetGenericInterface(vr::IVRTrackedCamera_Version, &eError);
			if (!steam.mCamera) {
				object_post(&ob, "failed to acquire camera -- is it enabled in the SteamVR settings?");
			}
			else {
				vr::EVRTrackedCameraError camError;
				bool bHasCamera = false;

				camError = steam.mCamera->HasCamera(vr::k_unTrackedDeviceIndex_Hmd, &bHasCamera);
				if (camError != vr::VRTrackedCameraError_None || !bHasCamera) {
					object_post(&ob, "No Tracked Camera Available! (%s)\n", steam.mCamera->GetCameraErrorNameFromEnum(camError));
					steam.mCamera = 0;
				}

				//if (use_camera) video_start();
			}
		}
		VR_DEBUG_POST("steam connected");

		driver = ps_steam;
		return true;
	}

	void steam_disconnect() {
		if (steam.hmd) {
			VR_DEBUG_POST("steam disconnect");

			release_gpu_resources();

			// TODO enable video
			//video_stop();
			
			//vr::VR_Shutdown();

			steam.hmd = 0;
		}
	}


	void steam_configure() {
		VR_DEBUG_POST("steam configure");
		if (!steam.hmd) {
			object_error(&ob, "no SteamVR session to configure");
			return;
		}
		t_atom a[6];
		t_symbol * display_name = steam_get_tracked_device_name(steam.hmd, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String);
		t_symbol * driver_name = steam_get_tracked_device_name(steam.hmd, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String);
		atom_setsym(a, display_name);
		outlet_anything(outlet_msg, gensym("display"), 1, a);
		atom_setsym(a, driver_name);
		outlet_anything(outlet_msg, gensym("driver"), 1, a);

		// determine the recommended texture size for scene capture:
		uint32_t dim[2];
		steam.hmd->GetRecommendedRenderTargetSize(&dim[0], &dim[1]);
		fbo_dim[0] = dim[0] * 2; // side-by-side
		fbo_dim[1] = dim[1];

		// maybe never: support disabling tracking options via ovr_ConfigureTracking()

		VR_DEBUG_POST("steam configured");
	}



	// TODO
	bool steam_create_gpu_resources() {
		VR_DEBUG_POST("steam_create_gpu_resources");
		if (!steam.hmd) return false;

		glGenTextures(1, &steam.fbo_texture_id);
		glGenRenderbuffersEXT(1, &rbo_id);

		VR_DEBUG_POST("fbo %d rbo %d tex %d", fbo_id, rbo_id, steam.fbo_texture_id);

		GLint previousFBO;	// make sure we pop out to the right FBO
		glGetIntegerv(GL_FRAMEBUFFER_BINDING_EXT, &previousFBO);
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo_id);
		{
			glBindTexture(GL_TEXTURE_2D, steam.fbo_texture_id);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, fbo_dim[0], fbo_dim[1], 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
			glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, steam.fbo_texture_id, 0);
			// TODO: is rbo actually necessary?
			glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, rbo_id);
			glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT, fbo_dim[0], fbo_dim[1]);
			glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, rbo_id);
			// check FBO status
			GLenum status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
			if (status != GL_FRAMEBUFFER_COMPLETE_EXT) {
				object_error(&ob, "failed to create Jitter FBO");
				return false;
			}
		}
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, previousFBO);


		// TODO mirror
		// TODO video camera

		//jit_gl_report_error("steam_create_gpu_resources done");

		return true;
	}

	void steam_release_gpu_resources() {
		// TODO
		if (steam.fbo_texture_id) {
			glDeleteTextures(1, &steam.fbo_texture_id);
			steam.fbo_texture_id = 0;
		}

		if (rbo_id) {
			glDeleteRenderbuffersEXT(1, &rbo_id);
			rbo_id = 0;
		}
	}

	void steam_bang() {
		if (!steam.hmd) return;

		t_atom a[6];

		vr::VREvent_t event;
		while (steam.hmd->PollNextEvent(&event, sizeof(event))) {
			switch (event.eventType) {
				case vr::VREvent_TrackedDeviceActivated:
				{
					atom_setlong(&a[0], event.trackedDeviceIndex);
					outlet_anything(outlet_msg, gensym("attached"), 1, a);
					//setupRenderModelForTrackedDevice(event.trackedDeviceIndex);
				}
				break;
				case vr::VREvent_TrackedDeviceDeactivated:
				{
					atom_setlong(&a[0], event.trackedDeviceIndex);
					outlet_anything(outlet_msg, gensym("detached"), 1, a);
				}
				break;
				//case vr::VREvent_TrackedDeviceUpdated: break;
				default: {
					// TODO: lots of interesting events in openvr.h
					// 
					//atom_setlong(&a[0], event.eventType);
					//outlet_anything(outlet_msg, gensym("event"), 1, a);
				}
			}
		}

		for (int i = 0; i < 2; i++) {

			float l, r, t, b;
			steam.hmd->GetProjectionRaw((vr::Hmd_Eye)i, &l, &r, &t, &b);

			//VR_DEBUG_POST("frustum l %f r %f t %f b %f", l, r, t, b);

			atom_setfloat(a + 0, l * near_clip);
			atom_setfloat(a + 1, r * near_clip);
			atom_setfloat(a + 2, -b * near_clip);
			atom_setfloat(a + 3, -t * near_clip);
			atom_setfloat(a + 4, near_clip);
			atom_setfloat(a + 5, far_clip);
			outlet_anything(outlet_eye[i], ps_frustum, 6, a);
		}

		// get the tracking data here
		vr::EVRCompositorError err = vr::VRCompositor()->WaitGetPoses(steam.pRenderPoseArray, vr::k_unMaxTrackedDeviceCount, NULL, 0);
		if (err != vr::VRCompositorError_None) {
			object_error(&ob, "WaitGetPoses error");
			return;
		}

		// TODO: should we ignore button presses etc. if so?
		bool inputCapturedByAnotherProcess = steam.hmd->IsInputFocusCapturedByAnotherProcess();

		// check each device slot:
		for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
			const vr::TrackedDevicePose_t& trackedDevicePose = steam.pRenderPoseArray[i];
			// if the device is actually connected:
			if (trackedDevicePose.bDeviceIsConnected) {
				
				switch (steam.hmd->GetTrackedDeviceClass(i)) {
				case vr::TrackedDeviceClass_HMD: {
					if (trackedDevicePose.bPoseIsValid) {
						t_symbol * id = ps_head;

						glm::mat4 mat = steam_output_tracked_device(id, trackedDevicePose);

						// use this to update cameras:
						for (int i = 0; i < 2; i++) {

							steam.head2eye_mat[i] = to_glm(steam.hmd->GetEyeToHeadTransform((vr::Hmd_Eye)i));
							eye_mat[i] = mat * steam.head2eye_mat[i];
							
							float l, r, t, b;
							steam.hmd->GetProjectionRaw((vr::Hmd_Eye)i, &l, &r, &t, &b);
							atom_setfloat(a + 0, l * near_clip);
							atom_setfloat(a + 1, r * near_clip);
							// TODO: check if this is right for Vive?
							//atom_setfloat(a + 2, -b * near_clip);
							//atom_setfloat(a + 3, -t * near_clip);
							atom_setfloat(a + 2, t * near_clip);
							atom_setfloat(a + 3, b * near_clip);
							atom_setfloat(a + 4, near_clip);
							atom_setfloat(a + 5, far_clip);
							outlet_anything(outlet_eye[i], ps_frustum, 6, a);
						}

					}
				} break;
				case vr::TrackedDeviceClass_Controller: {
					// check role to see if these are hands
					vr::ETrackedControllerRole role = steam.hmd->GetControllerRoleForTrackedDeviceIndex(i);
					switch (role) {
					case vr::TrackedControllerRole_LeftHand:
					case vr::TrackedControllerRole_RightHand: {
						//if (trackedDevicePose.eTrackingResult == vr::TrackingResult_Running_OK) {

						int hand = (role == vr::TrackedControllerRole_RightHand);
						steam.mHandControllerDeviceIndex[hand] = i;

						t_symbol * id = hand ? ps_right_hand : ps_left_hand;

						if (trackedDevicePose.bPoseIsValid) {

							steam_output_tracked_device(id, trackedDevicePose);

						}

						vr::VRControllerState_t cs;
						//OpenVR SDK 1.0.4 adds a 3rd arg for size
						steam.hmd->GetControllerState(i, &cs, sizeof(cs));

						atom_setsym(a + 0, ps_trigger);
						atom_setlong(a + 1, (cs.ulButtonTouched & vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger)) != 0);
						atom_setfloat(a + 2, cs.rAxis[1].x);
						outlet_anything(outlet_tracking, id, 3, a);

						atom_setsym(a + 0, ps_buttons);
						atom_setlong(a + 1, (cs.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_ApplicationMenu)) != 0);
						atom_setlong(a + 2, (cs.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_Grip)) != 0);
						outlet_anything(outlet_tracking, id, 3, a);

						atom_setsym(a + 0, ps_pad);
						atom_setlong(a + 1, (cs.ulButtonTouched & vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Touchpad)) != 0);
						atom_setfloat(a + 2, cs.rAxis[0].x);
						atom_setfloat(a + 3, cs.rAxis[0].y);
						atom_setlong(a + 4, (cs.ulButtonPressed & vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Touchpad)) != 0);
						outlet_anything(outlet_tracking, id, 5, a);

						//}
					}
															  break;
					default:
						break;
					}
				} break;
				case vr::TrackedDeviceClass_GenericTracker:
				{
					if (trackedDevicePose.bPoseIsValid) {

						t_symbol * id = _jit_sym_nothing;
						//Figure out which tracker it is using some kind of unique identifier
						vr::ETrackedPropertyError err = vr::TrackedProp_Success;
						char buf[vr::k_unMaxPropertyStringSize];
						if (vr::VRSystem()->GetStringTrackedDeviceProperty(i, vr::Prop_SerialNumber_String, buf, sizeof(buf), &err)) {
							t_symbol * id = gensym(buf);
						}

						steam_output_tracked_device(id, trackedDevicePose);

					}
				} break;
				default:
					break;
				}
			}
		}


		// video:
		//video_step();
	}

	// utility function for steam_bang()
	glm::mat4 steam_output_tracked_device(t_symbol * id, const vr::TrackedDevicePose_t& trackedDevicePose) {
		t_atom a[5];
		
		glm::mat4 mat = to_glm(trackedDevicePose.mDeviceToAbsoluteTracking);

		glm::vec3 p = glm::vec3(mat[3]); // the translation component
		glm::quat q = glm::quat_cast(mat); // the orientation component
										   // adjusted to world space
		glm::mat4 world_mat = view_mat * mat;
		glm::vec3 p1 = glm::vec3(world_mat[3]); // the translation component
		glm::quat q1 = glm::quat_cast(world_mat); // the orientation component

		atom_setsym(a + 0, ps_tracked_position);
		atom_setfloat(a + 1, p.x);
		atom_setfloat(a + 2, p.y);
		atom_setfloat(a + 3, p.z);
		outlet_anything(outlet_tracking, id, 4, a);

		atom_setsym(a + 0, ps_tracked_quat);
		atom_setfloat(a + 1, q.x);
		atom_setfloat(a + 2, q.y);
		atom_setfloat(a + 3, q.z);
		atom_setfloat(a + 4, q.w);
		outlet_anything(outlet_tracking, id, 5, a);

		atom_setsym(a + 0, _jit_sym_position);
		atom_setfloat(a + 1, p1.x);
		atom_setfloat(a + 2, p1.y);
		atom_setfloat(a + 3, p1.z);
		outlet_anything(outlet_tracking, id, 4, a);

		atom_setsym(a + 0, _jit_sym_quat);
		atom_setfloat(a + 1, q1.x);
		atom_setfloat(a + 2, q1.y);
		atom_setfloat(a + 3, q1.z);
		atom_setfloat(a + 4, q1.w);
		outlet_anything(outlet_tracking, id, 5, a);

		// velocities:
		// TODO: check if these are in tracking space
		glm::vec3 vel = to_glm(trackedDevicePose.vVelocity);
		glm::vec3 angvel = to_glm(trackedDevicePose.vAngularVelocity);
		// rotated into world space (TODO is this appropriate? rotate or unrotate?)
		vel = quat_rotate(view_quat, vel);
		angvel = quat_rotate(view_quat, angvel);

		atom_setsym(a + 0, ps_velocity);
		atom_setfloat(a + 1, vel.x);
		atom_setfloat(a + 2, vel.y);
		atom_setfloat(a + 3, vel.z);
		outlet_anything(outlet_tracking, id, 4, a);

		atom_setsym(a + 0, ps_angular_velocity);
		atom_setfloat(a + 1, angvel.x);
		atom_setfloat(a + 2, angvel.y);
		atom_setfloat(a + 3, angvel.z);
		outlet_anything(outlet_tracking, id, 4, a);

		return mat;
	}


	bool steam_submit_texture(GLuint input_texture_id, t_atom_long input_texture_dim[2]) {
		if (!steam.hmd) return false;

		// main difference here with oculus is that we have to allocate the texture
		// whereas with oculus, the driver gives us a texture (the textureChain stuff)

		// TODO: check success
		if (!fbo_copy_texture(input_texture_id, input_texture_dim,
			fbo_id, steam.fbo_texture_id, fbo_dim, false)) {
			object_error(&ob, "problem copying texture");
			return false;
		}

		vr::EVRCompositorError err;
		//GraphicsAPIConvention enum was renamed to TextureType in OpenVR SDK 1.0.5
		// TODO: expose different colour options as attributes?
		vr::Texture_t vrTexture = { (void*)steam.fbo_texture_id, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };

		vr::VRTextureBounds_t leftBounds = { 0.f, 0.f, 0.5f, 1.f };
		vr::VRTextureBounds_t rightBounds = { 0.5f, 0.f, 1.f, 1.f };

		err = vr::VRCompositor()->Submit(vr::Eye_Left, &vrTexture, &leftBounds);
		switch (err) {
		case 0:
			break;
		case 1:
			object_error(&ob, "submit error: Request failed.");
			break;
		case 100:
			object_error(&ob, "submit error: Incompatible version.");
			break;
		case 101:
			object_error(&ob, "submit error: Do not have focus.");
			break;
		case 102:
			object_error(&ob, "submit error: Invalid texture.");
			break;
		case 103:
			object_error(&ob, "submit error: Is not scene application.");
			break;
		case 104:
			object_error(&ob, "submit error: Texture is on wrong device.");
			break;
		case 105:
			object_error(&ob, "submit error: Texture uses unsupported format.");
			break;
		case 106:
			object_error(&ob, "submit error: Shared textures not supported.");
			break;
		case 107:
			object_error(&ob, "submit error: Index out of range.");
			break;
		case 108:
			object_error(&ob, "submit error: Already submitted.");
			break;
		default:
			object_error(&ob, "submit error: other");
			break;
		}

		err = vr::VRCompositor()->Submit(vr::Eye_Right, &vrTexture, &rightBounds);
		switch (err) {
		case 0:
			break;
		case 1:
			object_error(&ob, "submit error: Request failed.");
			break;
		case 100:
			object_error(&ob, "submit error: Incompatible version.");
			break;
		case 101:
			object_error(&ob, "submit error: Do not have focus.");
			break;
		case 102:
			object_error(&ob, "submit error: Invalid texture.");
			break;
		case 103:
			object_error(&ob, "submit error: Is not scene application.");
			break;
		case 104:
			object_error(&ob, "submit error: Texture is on wrong device.");
			break;
		case 105:
			object_error(&ob, "submit error: Texture uses unsupported format.");
			break;
		case 106:
			object_error(&ob, "submit error: Shared textures not supported.");
			break;
		case 107:
			object_error(&ob, "submit error: Index out of range.");
			break;
		case 108:
			object_error(&ob, "submit error: Already submitted.");
			break;
		default:
			object_error(&ob, "submit error: other");
			break;
		}

		if (glfinishhack) {
			// is this necessary?
			glClearColor(0, 0, 0, 1);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			// openvr header recommends this after submit:
			glFlush();
			glFinish();

			// issue on openvr suggests only this is needed
			// https://github.com/ValveSoftware/openvr/issues/460
			// glMemoryBarrier(GL_TEXTURE_UPDATE_BARRIER_BIT);
		}
		return true;
	}


#endif // ifdef USE_DRIVERS
};

void vr_connect(Vr * x) { x->connect(); }
void vr_disconnect(Vr * x) { x->disconnect(); }
void vr_configure(Vr * x) { x->configure(); }
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


void vr_dest_changed(Vr * x) { x->dest_changed(); }
void vr_dest_closing(Vr * x) { x->dest_closing(); }
void vr_draw(Vr * x) {} // not used


t_max_err vr_connected_set(Vr *x, t_object *attr, long argc, t_atom *argv) {
	t_atom_long l = atom_getlong(argv);
	if (x->connected != l) {
		if (l) {
			x->connect();
		}
		else {
			x->disconnect();
		}
	}
	return 0;
}

t_max_err vr_driver_set(Vr *x, t_object *attr, long argc, t_atom *argv) {
	t_symbol * l = atom_getsym(argv);
	// aliases:
	if (l == gensym("steam") || l == gensym("steamvr") || l == gensym("vive") || l == gensym("htcvive")) {
		l = ps_steam;
	}
	// apply:
	if (x->driver != l) {
		if (x->connected) {
			// auto re-connect
			x->disconnect();
			x->driver = l;
			x->connect();
		}
		else {
			// don't auto-connect. 
			// this guard necessary so that the initial constructor doesn't attempt to connect to early
			x->driver = l;
		}
	}
	return 0;
}

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
		t_symbol * drawto = _jit_sym_nothing;
		long attrstart = max_jit_attr_args_offset(argc, argv);
		if (attrstart && argv) {
			
			jit_atom_arg_getsym(&drawto, 0, attrstart, argv);
		}
		x = new (x)Vr(drawto);
		// apply attrs:
		attr_args_process(x, (short)argc, argv);
		x->attrs_ready = 1;
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
	ps_frustum = gensym("frustum");
	ps_tracked_position = gensym("tracked_position");
	ps_tracked_quat = gensym("tracked_quat");

	ps_head = gensym("head");
	ps_left_hand = gensym("left_hand");
	ps_right_hand = gensym("right_hand");

	ps_velocity = gensym("velocity");
	ps_angular_velocity = gensym("angular_velocity");
	ps_trigger = gensym("trigger");
	ps_hand_trigger = gensym("hand_trigger");
	ps_pad = gensym("pad");
	ps_buttons = gensym("buttons");

	ps_oculus = gensym("oculus");
	ps_steam = gensym("steam");

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
	jit_class_addmethod(this_class, (method)(vr_draw), "ob3d_draw", A_CANT, 0L);
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
	
	// steam-specific:
	//class_addmethod(this_class, (method)vr_vibrate, "vibrate", A_GIMME, 0);
	//class_addmethod(this_class, (method)vr_battery, "battery", A_GIMME, 0);
	
	CLASS_ATTR_FLOAT(this_class, "near_clip", 0, Vr, near_clip);
	CLASS_ATTR_ACCESSORS(this_class, "near_clip", NULL, vr_near_clip_set);
	CLASS_ATTR_FLOAT(this_class, "far_clip", 0, Vr, far_clip);
	CLASS_ATTR_ACCESSORS(this_class, "far_clip", NULL, vr_far_clip_set);

	CLASS_ATTR_LONG(this_class, "connected", 0, Vr, connected);
	CLASS_ATTR_ACCESSORS(this_class, "connected", NULL, vr_connected_set);
	CLASS_ATTR_STYLE(this_class, "connected", 0, "onoff");


	CLASS_ATTR_LONG(this_class, "glfinishhack", 0, Vr, glfinishhack);
	CLASS_ATTR_STYLE(this_class, "glfinishhack", 0, "onoff");

	CLASS_ATTR_LONG(this_class, "oculus_available", ATTR_SET_OPAQUE | ATTR_SET_OPAQUE_USER, Vr, oculus_available);
	CLASS_ATTR_STYLE(this_class, "oculus_available", 0, "onoff");
	CLASS_ATTR_LONG(this_class, "steam_available", ATTR_SET_OPAQUE | ATTR_SET_OPAQUE_USER, Vr, steam_available);
	CLASS_ATTR_STYLE(this_class, "steam_available", 0, "onoff");

	CLASS_ATTR_SYM(this_class, "driver", 0, Vr, driver);
	CLASS_ATTR_ACCESSORS(this_class, "driver", NULL, vr_driver_set);
	CLASS_ATTR_LONG(this_class, "preferred_driver_only", 0, Vr, preferred_driver_only);
	CLASS_ATTR_STYLE(this_class, "preferred_driver_only", 0, "onoff");

	
	class_register(CLASS_BOX, this_class);
}
