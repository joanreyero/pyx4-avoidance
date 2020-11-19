import numpy as np
from matchedFilters import MatchedFilter

def get_activation(flow, mf):
    """Get the activation of an avoidance neuron.

    Args:
        flow (np.ndarray): optic flow array
        mf (np.ndarray): matched filter array

    Returns:
        float: activation
    """
    return np.sum(flow * mf) / flow.size
    