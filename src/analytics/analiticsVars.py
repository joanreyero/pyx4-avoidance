
POSITION = 'distance'
VELOCITY = 'velocity'
STATE = 'pyx4_state'
FLOW = 'optic_flow'
FLOW_45 = 'optic_flow_c45'
FLOW_N45 = 'optic_flow_cn45'
ACTIVATION = 'activation'
ACTIVATION_GRAD = 'activation_gradient'
ACTIVATION_45 = 'activation_c45'
ACTIVATION_N45 = 'activation_cn45'
FOVX = 'fov_x'

POSITION_TOPIC = '/mavros/local_position/pose'
VELOCITY_TOPIC = '/mavros/local_position/velocity_local'
STATE_TOPIC = '/pyx4_node/pyx4_state'
FLOW_TOPIC =  '/pyx4_avoidance_node/optic_flow'
ACTIVATION_TOPIC =  '/pyx4_avoidance_node/activation'
FLOW_45_TOPIC =  '/pyx4_avoidance_node/optic_flow_c45'
ACTIVATION_45_TOPIC =  '/pyx4_avoidance_node/activation_c45'
FLOW_N45_TOPIC =  '/pyx4_avoidance_node/optic_flow_cn45'
ACTIVATION_N45_TOPIC =  '/pyx4_avoidance_node/activation_cn45'


TOPICS_TO_LABELS = {
    POSITION_TOPIC: POSITION,
    VELOCITY_TOPIC: VELOCITY,
    STATE_TOPIC: STATE,
    FLOW_TOPIC: FLOW,
    ACTIVATION_TOPIC: ACTIVATION,
    FLOW_45_TOPIC: FLOW_45,
    ACTIVATION_45_TOPIC: ACTIVATION_45,
    FLOW_N45_TOPIC: FLOW_N45,
    ACTIVATION_N45_TOPIC: ACTIVATION_N45
}

TOPICS = [k for k, v in TOPICS_TO_LABELS.iteritems()]

LABELS_TO_TOPICS = {v: k for k, v in 
                    TOPICS_TO_LABELS.iteritems()}
