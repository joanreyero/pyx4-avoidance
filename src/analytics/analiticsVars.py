
POSITION = 'distance'
VELOCITY = 'velocity'
STATE = 'pyx4_state'
FLOW = 'optic_flow'
ACTIVATION = 'activation'
FOVX = 'fov_x'

POSITION_TOPIC = '/mavros/local_position/pose'
VELOCITY_TOPIC = '/mavros/local_position/velocity_local'
STATE_TOPIC = '/pyx4_node/pyx4_state'
FLOW_TOPIC =  '/pyx4_avoidance_node/optic_flow'
ACTIVATION_TOPIC =  '/pyx4_avoidance_node/activation'


TOPICS_TO_LABELS = {
    POSITION_TOPIC: POSITION,
    VELOCITY_TOPIC: VELOCITY,
    STATE_TOPIC: STATE,
    FLOW_TOPIC: FLOW,
    ACTIVATION_TOPIC: ACTIVATION
}

TOPICS = [k for k, v in TOPICS_TO_LABELS.iteritems()]

LABELS_TO_TOPICS = {v: k for k, v in 
                    TOPICS_TO_LABELS.iteritems()}
