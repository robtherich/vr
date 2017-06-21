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
static t_symbol * ps_thumbstick;
static t_symbol * ps_buttons;

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

glm::quat from_ovr(ovrQuatf const q) {
	return glm::quat(q.x, q.y, q.z, q.w);
}

glm::vec3 from_ovr(ovrVector3f const v) {
	return glm::vec3(v.x, v.y, v.z);
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
	void * outlet_tex;
	
	// attrs:
	float near_clip = 0.15f;
	float far_clip = 100.f;
	
	// configuration:
	int connected = 0;
	int dest_ready = 0;

	glm::vec3 view_position;
	glm::quat view_quat;

	// guts:
	
	// FBO & texture that the scene is copied into
	// (we can't submit the jit_gl_texture directly)
	GLuint fbo_id, rbo_id, fbo_texture_id;
	t_atom_long fbo_texture_dim[2];

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
		long long frameIndex;
		double sensorSampleTime;    // sensorSampleTime is fed into the layer later
		int max_fov = 0; // use default field of view; set to 1 for maximum field of view
		float pixel_density = 1.f;
		int tracking_level = (int)ovrTrackingOrigin_FloorLevel;
	} oculus;

	struct {
		vr::IVRSystem *	hmd;
		t_symbol * driver;
		t_symbol * display;
		vr::TrackedDevicePose_t pRenderPoseArray[vr::k_unMaxTrackedDeviceCount];
		glm::mat4 mDevicePose[vr::k_unMaxTrackedDeviceCount];
		glm::mat4 mHMDPose;
		glm::mat4 m_mat4viewEye[2];
		glm::mat4 m_mat4projectionEye[2];

		vr::IVRRenderModels * mRenderModels;
		vr::IVRTrackedCamera * mCamera;
		vr::TrackedCameraHandle_t m_hTrackedCamera;
		uint32_t	m_nCameraFrameWidth;
		uint32_t	m_nCameraFrameHeight;
		uint32_t	m_nCameraFrameBufferSize;
		uint32_t	m_nLastFrameSequence;
		uint8_t		* m_pCameraFrameBuffer;
		vr::EVRTrackedCameraFrameType frametype;
	} vive;
	
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

		// TODO driver specific
		// try to see if the Oculus driver is available:
		if (!(oculusrift_init() && oculus_connect())) return;
		
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
			oculus_configure();
			// get recommended texture dim & output
			// initialize camera matrices, frusta, etc. & output
		}

		// output recommended texture dim:
		atom_setlong(a + 0, fbo_texture_dim[0]);
		atom_setlong(a + 1, fbo_texture_dim[1]);
		outlet_anything(outlet_msg, _jit_sym_dim, 2, a);

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

		// TODO driver specific:
		oculus_create_gpu_resources();
		
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

		// TODO driver specific
		oculus_release_gpu_resources();
	}
	
	// poll HMD for events
	// most importantly, it picks up the current HMD pose
	// this should be the *last* thing to happen before rendering the scene
	// nothing time-consuming should happen between bang() and render
	void bang() {
		// if not connected, possibly, poll for availability?
		if (!connected) return;
		
		// get desired view matrix (from @position and @quat attrs)
		object_attr_getfloat_array(this, _jit_sym_position, 3, &view_position.x);
		object_attr_getfloat_array(this, _jit_sym_quat, 4, &view_quat.x);
		glm::mat4 modelview_mat = glm::translate(glm::mat4(1.0f), view_position) * mat4_cast(quat_from_jitter(view_quat));
		


		// TODO: video (or separate message for this?)
		
		// TODO: driver poll events
		oculus_bang();

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

		// get input properties:
		GLuint input_texture_id = object_attr_getlong(jit_texture, ps_glid);
		// get input texture dimensions
		t_atom_long input_texture_dim[2];
		object_attr_getlong_array(jit_texture, _jit_sym_dim, 2, input_texture_dim);

		
		if (connected) {
			// submit it to the driver
			if (!fbo_id) {
				// TODO try to allocate FBO for copying Jitter texture to driver?
				// or just bug out at this point?
				object_error(&ob, "no fbo yet");
				return;	// no texture to copy from.
			}

			// TODO driver specific
			if (!oculus_submit_texture(input_texture_id, input_texture_dim)) {
				object_error(&ob, "problem submitting texture");
			}
		}
		
		t_atom a[1];
		if (connected) {
			// TODO: use mirror textures here if desired?
		}
		atom_setsym(a, intexture);
		outlet_anything(outlet_tex, ps_jit_gl_texture, 1, a);
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

			/*
			glClearColor(0, 0, 0, 1);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			
			glColor4f(1.0, 0.0, 1.0, 1.0);
*/
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

	void oculus_configure() {
		if (!oculus.session) {
			object_error(&ob, "no Oculus session to configure");
			return;
		}

		t_atom a[2];

		// maybe never: support disabling tracking options via ovr_ConfigureTracking()

		oculus.hmd = ovr_GetHmdDesc(oculus.session);
		// Use hmd members and ovr_GetFovTextureSize() to determine graphics configuration

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
		fbo_texture_dim[0] = recommenedTex0Size.w + recommenedTex1Size.w; // side-by-side
		fbo_texture_dim[1] = AL_MAX(recommenedTex0Size.h, recommenedTex1Size.h);

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
	}

	bool oculus_create_gpu_resources() {
		if (!oculus.session) return false;
		if (!oculus.textureChain) {
			ovrTextureSwapChainDesc desc = {};
			desc.Type = ovrTexture_2D;
			desc.ArraySize = 1;
			desc.Width = fbo_texture_dim[0];
			desc.Height = fbo_texture_dim[1];
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

			int length = 0;
			ovr_GetTextureSwapChainLength(oculus.session, oculus.textureChain, &length);

			// we can update the layer too here:
			oculus.layer.ColorTexture[0] = oculus.textureChain;
			oculus.layer.ColorTexture[1] = oculus.textureChain;

		}
		return true;
	}

	void oculus_release_gpu_resources() {
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

		}
		else {
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

			// get current head pose
			const ovrPosef& pose = ts.HeadPose.ThePose;
			// Computes offset eye poses based on headPose returned by ovrTrackingState.
			// use the tracking state to update the layers (part of how timewarp works)
			ovr_CalcEyePoses(pose, oculus.hmdToEyeViewOffset, oculus.layer.RenderPose);

			// update the camera poses/frusta accordingly
			for (int eye = 0; eye < 2; eye++) {

				// update the layer info too:
				oculus.layer.Fov[eye] = oculus.eyeRenderDesc[eye].Fov;
				oculus.layer.SensorSampleTime = oculus.sensorSampleTime;

				const glm::vec3 p = from_ovr(oculus.layer.RenderPose[eye].Position) + view_position;
				atom_setfloat(a + 0, p.x);
				atom_setfloat(a + 1, p.y);
				atom_setfloat(a + 2, p.z);
				outlet_anything(outlet_eye[eye], _jit_sym_position, 3, a);

				glm::quat eye_quat = from_ovr(oculus.layer.RenderPose[eye].Orientation) * view_quat;
				atom_setfloat(a + 0, eye_quat.x);
				atom_setfloat(a + 1, eye_quat.y);
				atom_setfloat(a + 2, eye_quat.z);
				atom_setfloat(a + 3, eye_quat.w);
				outlet_anything(outlet_eye[eye], _jit_sym_quat, 4, a);

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

			// Headset tracking data:
			{
				t_symbol * id = ps_head;
				
				// raw tracking data
				glm::vec3 p = from_ovr(pose.Position);
				glm::quat q = from_ovr(pose.Orientation);

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

				// adjusted to world space
				glm::vec3 p1 = p + view_position;
				glm::quat q1 = q * view_quat;

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
					glm::vec3 p = from_ovr(pose.Position);
					glm::quat q = from_ovr(pose.Orientation);

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

					// adjusted to world space
					glm::vec3 p1 = p + view_position;
					glm::quat q1 = q * view_quat;

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
					glm::vec3 vel = from_ovr(ts.HandPoses[i].LinearVelocity);
					glm::vec3 angvel = from_ovr(ts.HandPoses[i].AngularVelocity);

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

					atom_setsym(a + 0, ps_thumbstick);
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
		if (!copy_texture_to_fbo(input_texture_id, input_texture_dim, fbo_id, oculus_target_texture_id, fbo_texture_dim)) {
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

	static t_symbol * vive_get_tracked_device_name(vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL)
	{
		uint32_t unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, NULL, 0, peError);
		if (unRequiredBufferLen == 0) return _jit_sym_nothing;

		char *pchBuffer = new char[unRequiredBufferLen];
		unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, pchBuffer, unRequiredBufferLen, peError);
		t_symbol * sResult = gensym(pchBuffer);
		delete[] pchBuffer;
		return sResult;
	}

	void vive_configure() {
		if (!vive.hmd) return;

		t_symbol * display_name = vive_get_tracked_device_name(vive.hmd, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String);
		t_symbol * driver_name = vive_get_tracked_device_name(vive.hmd, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String);
		object_post(&ob, "display %s driver %s", display_name->s_name, driver_name->s_name);

		// determine the recommended texture size for scene capture:
		uint32_t dim[2];
		vive.hmd->GetRecommendedRenderTargetSize(&dim[0], &dim[1]);
		fbo_texture_dim[0] = dim[0]*2; // side-by-side
		fbo_texture_dim[1] = dim[1];
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
	ps_thumbstick = gensym("thumbstick");
	ps_buttons = gensym("buttons");

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
