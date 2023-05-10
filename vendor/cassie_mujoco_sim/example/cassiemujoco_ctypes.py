import ctypes

# Path: vendor/cassie_mujoco_sim/libcassiemujoco.so

# Load library
libcassiesim = ctypes.cdll.LoadLibrary("vendor/cassie_mujoco_sim/libcassiemujoco.so")

# Use ctypes to load the library
# void cassie_mujoco_init(char* path)
cassie_mujoco_init = libcassiesim.cassie_mujoco_init
libcassiesim.cassie_mujoco_init.argtypes = [ctypes.c_char_p]
libcassiesim.cassie_mujoco_init.restype = None

# cassie_out_t* cassie_sim_init()
cassie_sim_init = libcassiesim.cassie_sim_init
libcassiesim.cassie_sim_init.argtypes = None
libcassiesim.cassie_sim_init.restype = ctypes.c_void_p

# struct battery_out_t
class battery_out_t(ctypes.Structure):
    _fields_ = [("dataGood", ctypes.c_bool),
                ("stateOfCharge", ctypes.c_double),
                ("voltage", ctypes.c_double * 12),
                ("current", ctypes.c_double),
                ("temperature", ctypes.c_double * 4)]
    
class cassie_joint_out_t(ctypes.Structure):
    _fields_ = [("position", ctypes.c_double),
                ("velocity", ctypes.c_double)]
    
# Define the struct elmo_out_t
class elmo_out_t(ctypes.Structure):
    _fields_ = [("statusWord", ctypes.c_ushort),
                ("position", ctypes.c_double),
                ("velocity", ctypes.c_double),
                ("torque", ctypes.c_double),
                ("driveTemperature", ctypes.c_double),
                ("dcLinkVoltage", ctypes.c_double),
                ("torqueLimit", ctypes.c_double),
                ("gearRatio", ctypes.c_double)]

# Define the struct cassie_leg_out_t
class cassie_leg_out_t(ctypes.Structure):
    _fields_ = [("hipRollDrive", elmo_out_t),
                ("hipYawDrive", elmo_out_t),
                ("hipPitchDrive", elmo_out_t),
                ("kneeDrive", elmo_out_t),
                ("footDrive", elmo_out_t),
                ("shinJoint", cassie_joint_out_t),
                ("tarsusJoint", cassie_joint_out_t),
                ("footJoint", cassie_joint_out_t),
                ("medullaCounter", ctypes.c_ubyte),
                ("medullaCpuLoad", ctypes.c_ushort),
                ("reedSwitchState", ctypes.c_bool)]

# Define the struct radio_out_t
class radio_out_t(ctypes.Structure):
    _fields_ = [("radioReceiverSignalGood", ctypes.c_bool),
                ("receiverMedullaSignalGood", ctypes.c_bool),
                ("channel", ctypes.c_double * 16)]

# Define the struct target_pc_out_t
class target_pc_out_t(ctypes.Structure):
    _fields_ = [("etherCatStatus", ctypes.c_int * 6),
                ("etherCatNotifications", ctypes.c_int * 21),
                ("taskExecutionTime", ctypes.c_double),
                ("overloadCounter", ctypes.c_uint),
                ("cpuTemperature", ctypes.c_double)]

# Define the struct vectornav_out_t
class vectornav_out_t(ctypes.Structure):
    _fields_ = [("dataGood", ctypes.c_bool),
                ("vpeStatus", ctypes.c_ushort),
                ("pressure", ctypes.c_double),
                ("temperature", ctypes.c_double),
                ("magneticField", ctypes.c_double * 3),
                ("angularVelocity", ctypes.c_double * 3),
                ("linearAcceleration", ctypes.c_double * 3),
                ("orientation", ctypes.c_double * 4)]

# Define the struct cassie_pelvis_out_t
class cassie_pelvis_out_t(ctypes.Structure):
    _fields_ = [("targetPc", target_pc_out_t),
                ("battery", battery_out_t),
                ("radio", radio_out_t),
                ("vectorNav", vectornav_out_t),
                ("medullaCounter", ctypes.c_ubyte),
                ("medullaCpuLoad", ctypes.c_ushort),
                ("bleederState", ctypes.c_bool),
                ("leftReedSwitchState", ctypes.c_bool),
                ("rightReedSwitchState", ctypes.c_bool),
                ("vtmTemperature", ctypes.c_double)]

# Define the 'DiagnosticCodes' type
DiagnosticCodes = ctypes.c_short

# Define the 'cassie_out_t' struct
class cassie_out_t(ctypes.Structure):
    _fields_ = [
        ('pelvis', cassie_pelvis_out_t),
        ('leftLeg', cassie_leg_out_t),
        ('rightLeg', cassie_leg_out_t),
        ('isCalibrated', ctypes.c_bool),
        ('messages', DiagnosticCodes * 4),
    ]

# Define the struct state_battery_out_t
class state_battery_out_t(ctypes.Structure):
    _fields_ = [("stateOfCharge", ctypes.c_double),
                ("current", ctypes.c_double)]

# Define the struct state_foot_out_t
class state_foot_out_t(ctypes.Structure):
    _fields_ = [("position", ctypes.c_double * 3),
                ("orientation", ctypes.c_double * 4),
                ("footRotationalVelocity", ctypes.c_double * 3),
                ("footTranslationalVelocity", ctypes.c_double * 3),
                ("toeForce", ctypes.c_double * 3),
                ("heelForce", ctypes.c_double * 3)]

# Define the struct state_joint_out_t
class state_joint_out_t(ctypes.Structure):
    _fields_ = [("position", ctypes.c_double * 6),
                ("velocity", ctypes.c_double * 6)]

# Define the struct state_motor_out_t
class state_motor_out_t(ctypes.Structure):
    _fields_ = [("position", ctypes.c_double * 10),
                ("velocity", ctypes.c_double * 10),
                ("torque", ctypes.c_double * 10)]

# Define the struct state_battery_out_t
class state_battery_out_t(ctypes.Structure):
    _fields_ = [("stateOfCharge", ctypes.c_double),
                ("current", ctypes.c_double)]

# Define the struct state_foot_out_t
class state_foot_out_t(ctypes.Structure):
    _fields_ = [("position", ctypes.c_double * 3),
                ("orientation", ctypes.c_double * 4),
                ("footRotationalVelocity", ctypes.c_double * 3),
                ("footTranslationalVelocity", ctypes.c_double * 3),
                ("toeForce", ctypes.c_double * 3),
                ("heelForce", ctypes.c_double * 3)]

# Define the struct state_joint_out_t
class state_joint_out_t(ctypes.Structure):
    _fields_ = [("position", ctypes.c_double * 6),
                ("velocity", ctypes.c_double * 6)]

# Define the struct state_motor_out_t
class state_motor_out_t(ctypes.Structure):
    _fields_ = [("position", ctypes.c_double * 10),
                ("velocity", ctypes.c_double * 10),
                ("torque", ctypes.c_double * 10)]

# Define the struct state_pelvis_out_t
class state_pelvis_out_t(ctypes.Structure):
    _fields_ = [("position", ctypes.c_double * 3),
                ("orientation", ctypes.c_double * 4),
                ("rotationalVelocity", ctypes.c_double * 3),
                ("translationalVelocity", ctypes.c_double * 3),
                ("translationalAcceleration", ctypes.c_double * 3),
                ("externalMoment", ctypes.c_double * 3),
                ("externalForce", ctypes.c_double * 3)]

# Define the struct state_radio_out_t
class state_radio_out_t(ctypes.Structure):
    _fields_ = [("channel", ctypes.c_double * 16),
                ("signalGood", ctypes.c_bool)]

# Define the struct state_terrain_out_t
class state_terrain_out_t(ctypes.Structure):
    _fields_ = [("height", ctypes.c_double),
                ("slope", ctypes.c_double * 2)]

# Define the struct state_out_t
class state_out_t(ctypes.Structure):
    _fields_ = [("pelvis", state_pelvis_out_t),
                ("leftFoot", state_foot_out_t),
                ("rightFoot", state_foot_out_t),
                ("terrain", state_terrain_out_t),
                ("motor", state_motor_out_t),
                ("joint", state_joint_out_t),
                ("radio", state_radio_out_t),
                ("battery", state_battery_out_t)]

# Define the struct state_pelvis_out_t
class state_pelvis_out_t(ctypes.Structure):
    _fields_ = [("position", ctypes.c_double * 3),
                ("orientation", ctypes.c_double * 4),
                ("rotationalVelocity", ctypes.c_double * 3),
                ("translationalVelocity", ctypes.c_double * 3),
                ("translationalAcceleration", ctypes.c_double * 3),
                ("externalMoment", ctypes.c_double * 3),
                ("externalForce", ctypes.c_double * 3)]

# Define the struct state_radio_out_t
class state_radio_out_t(ctypes.Structure):
    _fields_ = [("channel", ctypes.c_double * 16),
                ("signalGood", ctypes.c_bool)]

# Define the struct state_terrain_out_t
class state_terrain_out_t(ctypes.Structure):
    _fields_ = [("height", ctypes.c_double),
                ("slope", ctypes.c_double * 2)]

# Define the struct state_out_t
class state_out_t(ctypes.Structure):
    _fields_ = [("pelvis", state_pelvis_out_t),
                ("leftFoot", state_foot_out_t),
                ("rightFoot", state_foot_out_t),
                ("terrain", state_terrain_out_t),
                ("motor", state_motor_out_t),
                ("joint", state_joint_out_t),
                ("radio", state_radio_out_t),
                ("battery", state_battery_out_t)]

# TODO: cassie_sim_t, cassie_user_in_t, cassie_sim_t, pd_in_t, cassie_state_t
# void cassie_sim_step(cassie_sim_t *c, cassie_out_t *y, const cassie_user_in_t *u)
cassie_sim_step = libcassiesim.cassie_sim_step
libcassiesim.cassie_sim_step.argtypes = [ctypes.c_void_p, ctypes.POINTER(cassie_out_t), ctypes.c_void_p]
libcassiesim.cassie_sim_step.restype = None

# void cassie_sim_step_pd(cassie_sim_t *c, state_out_t *y, const pd_in_t *u)
cassie_sim_step_pd = libcassiesim.cassie_sim_step_pd
libcassiesim.cassie_sim_step_pd.argtypes = [ctypes.c_void_p, ctypes.POINTER(state_out_t), ctypes.c_void_p]
libcassiesim.cassie_sim_step_pd.restype = None

# void cassie_get_state(const cassie_sim_t *c, cassie_state_t *s)
cassie_get_state = libcassiesim.cassie_get_state
libcassiesim.cassie_get_state.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
libcassiesim.cassie_get_state.restype = None

# void cassie_set_state(cassie_sim_t *c, const cassie_state_t *s)
cassie_set_state = libcassiesim.cassie_set_state
libcassiesim.cassie_set_state.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
libcassiesim.cassie_set_state.restype = None

# double *cassie_sim_time(cassie_sim_t *c)
cassie_sim_time = libcassiesim.cassie_sim_time
libcassiesim.cassie_sim_time.argtypes = [ctypes.c_void_p]
libcassiesim.cassie_sim_time.restype = ctypes.POINTER(ctypes.c_double)

# double *cassie_sim_qpos(cassie_sim_t *c)
cassie_sim_qpos = libcassiesim.cassie_sim_qpos
libcassiesim.cassie_sim_qpos.argtypes = [ctypes.c_void_p]
libcassiesim.cassie_sim_qpos.restype = ctypes.POINTER(ctypes.c_double)

# double *cassie_sim_qvel(cassie_sim_t *c)
cassie_sim_qvel = libcassiesim.cassie_sim_qvel
libcassiesim.cassie_sim_qvel.argtypes = [ctypes.c_void_p]
libcassiesim.cassie_sim_qvel.restype = ctypes.POINTER(ctypes.c_double)

# void cassie_sim_hold(cassie_sim_t *c)
cassie_sim_hold = libcassiesim.cassie_sim_hold
libcassiesim.cassie_sim_hold.argtypes = [ctypes.c_void_p]
libcassiesim.cassie_sim_hold.restype = None

# void cassie_sim_release(cassie_sim_t *c)
cassie_sim_release = libcassiesim.cassie_sim_release
libcassiesim.cassie_sim_release.argtypes = [ctypes.c_void_p]
libcassiesim.cassie_sim_release.restype = None

# void cassie_sim_apply_force(cassie_sim_t *c, double xfrc[6], int body)
cassie_sim_apply_force = libcassiesim.cassie_sim_apply_force
libcassiesim.cassie_sim_apply_force.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_double), ctypes.c_int]
libcassiesim.cassie_sim_apply_force.restype = None

# void cassie_sim_clear_forces(cassie_sim_t *c)
cassie_sim_clear_forces = libcassiesim.cassie_sim_clear_forces
libcassiesim.cassie_sim_clear_forces.argtypes = [ctypes.c_void_p]
libcassiesim.cassie_sim_clear_forces.restype = None

# void cassie_sim_free(cassie_sim_t *c)
cassie_sim_free = libcassiesim.cassie_sim_free
libcassiesim.cassie_sim_free.argtypes = [ctypes.c_void_p]
libcassiesim.cassie_sim_free.restype = None

# cassie_vis_t *cassie_vis_init()
cassie_vis_init = libcassiesim.cassie_vis_init
libcassiesim.cassie_vis_init.argtypes = None
libcassiesim.cassie_vis_init.restype = ctypes.c_void_p

# bool cassie_vis_draw(cassie_vis_t *v, cassie_sim_t *c)
cassie_vis_draw = libcassiesim.cassie_vis_draw
libcassiesim.cassie_vis_draw.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
libcassiesim.cassie_vis_draw.restype = ctypes.c_bool

# bool cassie_vis_valid(cassie_vis_t *v)
cassie_vis_valid = libcassiesim.cassie_vis_valid
libcassiesim.cassie_vis_valid.argtypes = [ctypes.c_void_p]
libcassiesim.cassie_vis_valid.restype = ctypes.c_bool

# void cassie_vis_free(cassie_vis_t *v)
cassie_vis_free = libcassiesim.cassie_vis_free
libcassiesim.cassie_vis_free.argtypes = [ctypes.c_void_p]
libcassiesim.cassie_vis_free.restype = None

# cassie_state_t *cassie_state_alloc()
cassie_state_alloc = libcassiesim.cassie_state_alloc
libcassiesim.cassie_state_alloc.argtypes = None
libcassiesim.cassie_state_alloc.restype = ctypes.c_void_p

# double *cassie_state_time(cassie_state_t *s)
cassie_state_time = libcassiesim.cassie_state_time
libcassiesim.cassie_state_time.argtypes = [ctypes.c_void_p]
libcassiesim.cassie_state_time.restype = ctypes.POINTER(ctypes.c_double)

# double *cassie_state_qpos(cassie_state_t *s)
cassie_state_qpos = libcassiesim.cassie_state_qpos
libcassiesim.cassie_state_qpos.argtypes = [ctypes.c_void_p]
libcassiesim.cassie_state_qpos.restype = ctypes.POINTER(ctypes.c_double)

# double *cassie_state_qvel(cassie_state_t *s)
cassie_state_qvel = libcassiesim.cassie_state_qvel
libcassiesim.cassie_state_qvel.argtypes = [ctypes.c_void_p]
libcassiesim.cassie_state_qvel.restype = ctypes.POINTER(ctypes.c_double)

# void cassie_state_free(cassie_state_t *s)
cassie_state_free = libcassiesim.cassie_state_free
libcassiesim.cassie_state_free.argtypes = [ctypes.c_void_p]
libcassiesim.cassie_state_free.restype = None

# int udp_init_client(const char *remote_addr_str, const char *remote_port_str,
udp_init_client = libcassiesim.udp_init_client
libcassiesim.udp_init_client.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
libcassiesim.udp_init_client.restype = ctypes.c_int

class packet_header_info_t(ctypes.Structure):
    _fields_ = [
        ('seq_num_out', ctypes.c_char),
        ('seq_num_in_last', ctypes.c_char),
        ('delay', ctypes.c_char),
        ('seq_num_in_diff', ctypes.c_char),
    ]

# Define pd_motor_in_t struct
class pd_motor_in_t(ctypes.Structure):
    _fields_ = [("torque", ctypes.c_double * 5),
                ("pTarget", ctypes.c_double * 5),
                ("dTarget", ctypes.c_double * 5),
                ("pGain", ctypes.c_double * 5),
                ("dGain", ctypes.c_double * 5)]

# Define pd_task_in_t struct
class pd_task_in_t(ctypes.Structure):
    _fields_ = [("torque", ctypes.c_double * 6),
                ("pTarget", ctypes.c_double * 6),
                ("dTarget", ctypes.c_double * 6),
                ("pGain", ctypes.c_double * 6),
                ("dGain", ctypes.c_double * 6)]

# Define pd_leg_in_t struct
class pd_leg_in_t(ctypes.Structure):
    _fields_ = [("taskPd", pd_task_in_t),
                ("motorPd", pd_motor_in_t)]

# Define pd_in_t struct
class pd_in_t(ctypes.Structure):
    _fields_ = [("leftLeg", pd_leg_in_t),
                ("rightLeg", pd_leg_in_t),
                ("telemetry", ctypes.c_double * 9)]

# void pack_cassie_user_in_t(const cassie_user_in_t *bus, unsigned char *bytes);
pack_cassie_user_in_t = libcassiesim.pack_cassie_user_in_t
libcassiesim.pack_cassie_user_in_t.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_ubyte)]
libcassiesim.pack_cassie_user_in_t.restype = None

# ssize_t send_packet(int sock, void *sendbuf, size_t sendlen, struct sockaddr *dst_addr, socklen_t addrlen)
send_packet = libcassiesim.send_packet
libcassiesim.send_packet.argtypes = [ctypes.c_int, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_size_t]
libcassiesim.send_packet.restype = ctypes.c_ssize_t

# void pack_pd_in_t(const pd_in_t *bus, unsigned char *bytes);
pack_pd_in_t = libcassiesim.pack_pd_in_t
libcassiesim.pack_pd_in_t.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_ubyte)]
libcassiesim.pack_pd_in_t.restype = None

# ssize_t get_newest_packet(int sock, void *recvbuf, size_t recvlen, struct sockaddr *src_addr, socklen_t *addrlen)
get_newest_packet = libcassiesim.get_newest_packet
libcassiesim.get_newest_packet.argtypes = [ctypes.c_int, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.POINTER(ctypes.c_size_t)]
libcassiesim.get_newest_packet.restype = ctypes.c_ssize_t

# void process_packet_header(packet_header_info_t *info, const unsigned char *header_in, unsigned char *header_out)
process_packet_header = libcassiesim.process_packet_header
libcassiesim.process_packet_header.argtypes = [ctypes.POINTER(packet_header_info_t), ctypes.POINTER(ctypes.c_ubyte), ctypes.POINTER(ctypes.c_ubyte)]
libcassiesim.process_packet_header.restype = None

# void unpack_cassie_out_t(const unsigned char *bytes, cassie_out_t *bus);
unpack_cassie_out_t = libcassiesim.unpack_cassie_out_t
libcassiesim.unpack_cassie_out_t.argtypes = [ctypes.POINTER(ctypes.c_ubyte), ctypes.c_void_p]
libcassiesim.unpack_cassie_out_t.restype = None

# void unpack_state_out_t(const unsigned char *bytes, state_out_t *bus);
unpack_state_out_t = libcassiesim.unpack_state_out_t
libcassiesim.unpack_state_out_t.argtypes = [ctypes.POINTER(ctypes.c_ubyte), ctypes.c_void_p]
libcassiesim.unpack_state_out_t.restype = None

# void udp_close(int sock)
udp_close = libcassiesim.udp_close
libcassiesim.udp_close.argtypes = [ctypes.c_int]
libcassiesim.udp_close.restype = None
