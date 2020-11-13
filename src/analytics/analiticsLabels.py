from analiticsVars import *


TOPICS_TO_LABELS = {
    '/mavros/local_position/pose': POSITION,
    '/mavros/local_position/velocity_local': VELOCITY,
    '/pyx4_node/pyx4_state': 'pyx4_state'
}

LABELS_TO_TOPICS = {v: k for k, v in 
                    TOPICS_TO_LABELS.iteritems()}


class AnalyticsLabels(object):
    """Generate labels for topics and vice-versa
    """

    @staticmethod
    def from_topics(topics):
        """Given a list of topics, return the labels

        Args:
            topics (iterable): list of topics

        Returns:
            iterable: list of labels
        """
        return[TOPICS_TO_LABELS[topic] for topic in topics]

    @staticmethod
    def from_labels(self, labels):
        """Given a list of labels, return the topics

        Args:
            labels (iterable): list of labels

        Returns:
            iterable: list of topics
        """
        return [LABELS_TO_TOPICS[label] for label in labels]

    @staticmethod
    def get_label(topic): return TOPICS_TO_LABELS[topic]
    
