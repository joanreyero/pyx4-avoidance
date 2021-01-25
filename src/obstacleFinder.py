from collections import deque
from camera_labels import *
import numpy as np


class ActivationDecisionMaker(object):
    
    def __init__(self, vel, min_init=5, min_decisions=1, min_gradient_constant=0.01, check_outliers=True, maxlen='1+', report=False):
        self.vel = vel
        
        # Minimum number of activations seen to initialise
        self.min_init = min_init
        # Minumum number of decisions True to say obstacle
        self.min_decisions = min_decisions

        self.min_gradient_constant = min_gradient_constant
        self.maxlen = self.get_maxlen(maxlen)
        
        self.setup()
        
        self.report = report
        self.last_activations = deque(self.mean + self.std * np.random.rand(10), maxlen=10)
        
        if self.report:
            # Initalise empty reporting
            self.report_activations = []
            self.report_decisions = []
            self.report_distance = []

        self.check_outliers = check_outliers
        
    def setup(self):
        # Obtained from the offline graphs using np.lstsq
        w_means, w_stds = 0.14254784, 0.02400344

        self.n = 0  # number of activations visited
        self.std = w_stds * self.vel  # Dynamic standard deviation
        self.mean = w_means * self.vel  # Dynamic mean
        self._init = False 

        # We need {min_decisions} to be True to make a decision
        self.decisions = deque(maxlen=self.maxlen)

        self.started = False

        self.min_gradient = self.min_gradient_constant * (1 + (self.vel - 1))

    def reset(self):
        self.setup()

    def start(self):
        self.started = True
        
    def check_init(self):
        """Check whether we can initialise
        """
        if self.n >= self.min_init:
            self._init = True 

    def get_maxlen(self, maxlen):
        if '+' in maxlen:
            return self.min_decisions + int(maxlen[:maxlen.find('+')])
        else:
            return self.min_decisions * int(maxlen[:maxlen.find('*')])

    def is_outlier(self, activation, report_cam=False, target_report_cam=C45):
        previous = np.array(self.last_activations) * 1.5
        percentile_75 = np.percentile(previous, 75)
        outlier_p = activation >= percentile_75 and activation > 0

        if report_cam == target_report_cam:
            print('\nCamera: ' + report_cam)
            print('  - 75th Percentile: ' + str(percentile_75))
            print('  - Activation: ' + str(activation))
            print('  - IS OUTLIER: ' + str(outlier_p) + '\n')

        return outlier_p
        
    def make_one_decision(self, report_cam=False, target_report_cam=C0,
                          only_report_when_decision=True):
        grads = np.gradient(np.array(self.last_activations))
        increasing = grads[np.where(grads > self.min_gradient)]
        decision = increasing.size >= 7

        if report_cam == target_report_cam:
            if not only_report_when_decision or decision:
                print('\nGradients that passed for camera ' + report_cam)
                print(increasing)
                # print('\nGradients that passed: ')
                # print(increasing)
                print('')
            
        return decision

    def make_decision(self, report_cam=False, target_report_cam=C45, 
                      only_report_decision=False):

        decisions = self.decisions
        last_decision = self.make_one_decision(
            report_cam=report_cam, 
            target_report_cam=target_report_cam,
            only_report_when_decision=only_report_decision
        )
        decisions.append(last_decision)

        return sum(decisions) >= self.min_decisions
    

    def step(self, activation, distance=False, report_cam=False):
        """Perform a checking step

        Args:
            activation (new activation): new activation
            distance (bool, optional): Distance (only for reporting). 
                                       Defaults to False.

        Returns:
            bool: decision
        """
        if self._init and self.started:
            # Check for outlier
            self.last_activations.append(activation)
            if (not self.is_outlier(activation, report_cam=False) or 
                not self.check_outliers):
                self.n += 1
                
                # Make decision
                decision = self.make_decision(report_cam=report_cam, 
                                              target_report_cam=C45,
                                              only_report_decision=False)
                self.decisions.append(decision)

                # For reporting                
                self.add_to_report(activation, distance, decision)
    
                return decision
   
        elif self.started:
            self.n += 1
            self.last_activations.append(activation)
            # Check whether we can initialise
            self.check_init()
            self.add_to_report(0, np.nan, np.nan)

        return False

    def add_to_report(self, activation, distance, decision):
        if self.report:
            self.report_activations.append(activation)
            self.report_distance.append(distance)

            if not decision or decision != np.nan:
                self.report_decisions.append(activation)
            else:
                self.report_decisions.append(np.nan)
            
        
