from collections import deque
import numpy as np


class ActivationDecisionMaker(object):
    
    def __init__(self, vel, min_init=10, min_decision=7, threshold_constant=2.5, report=False):
        """Initialise decision maker

        Args:
            vel (float): velocity
            min_init (int, optional): minimum activation values before 
                                      start. 
                                      Defaults to 20.
            min_decision (int, optional): minimum number to make decision. 
                                          Defaults to 6.
            threshold_constant (int, optional): number to multiply mean by 
                                                to get threshol. 
                                                Defaults to 1.
            report (bool, optional): keep a list of results to report. 
                                     Defaults to False.
        """
        self.vel = vel
        
        # Minimum number of activations seen to initialise
        self.min_init = min_init
        # Minumum number of decisions True to say obstacle
        self.min_decision = min_decision
        self.threshold_constant = threshold_constant
        self.setup()
        
        self.report = report
        
        if self.report:
            # Initalise empty reporting
            self.report_activations = []
            self.report_thresholds = []
            self.report_decisions = []
            self.report_distance = []
            self.report_mean = []

        
    def setup(self):
        # Obtained from the offline graphs using np.lstsq
        w_means, w_stds = 0.14254784, 0.02400344

        self.n = 0  # number of activations visited
        self.std = w_stds * self.vel  # Dynamic standard deviation
        self.mean = w_means * self.vel  # Dynamic mean
        self._init = False 

        # We need {min_decision} to be True to make a decision
        self.decisions = deque(maxlen=self.min_decision)

        # To find outliers. Comes from offline data
        self.noise_mean = self.vel * 0.14662427
        self.noise_std = self.vel * 0.10710734
        self.last_activations = deque(self.mean + self.std * np.random.rand(10), maxlen=10)
        self.started = False

    def reset(self):
        self.setup()

    def start(self):
        self.started = True
        
    def check_init(self):
        """Check whether we can initialise
        """
        if self.n >= self.min_init:
            self._init = True
            
    def update_stats(self, activation, n, mean, std):
        """Update the mean and standard deviation given an activation.

        Args:
            activation (float): the current activation
        """
        # Increase n
        n += 1
        # Save old var and mean
        old_var = std ** 2
        old_mean = mean

        # New mean: multiply by n-1 to get the previous sum,
        # add the new activation, and divide by n
        mean = (mean * (n - 1) + activation) / n
        # New var: 
        # https://math.stackexchange.com/questions/102978/incremental-computation-of-standard-deviation
        var = ((float((float(n) - 2)) / (float(n) - 1) * float(old_var)) + 
               1 / float(n) * (activation - old_mean) ** 2)
        
        
        return mean, np.sqrt(var)

        return mean, std
        
    def is_outlier_old(self, activation, m=3, report=False):
        """Find if it as outlier

        Args:
            activation (float): activation
            m (int, optional): number of stds away from mean. Defaults to 1.
            p (bool, optional): print or not. Defaults to False.

        Returns:
            bool: whether it is an outlier or not
        """
        dist_from_mean = np.abs(activation - self.noise_mean)
        
        if p and p == 'cam_0':
            print('Mean:       ', self.noise_mean)
            print('Activation: ', activation)
            print('Distance:   ', dist_from_mean)
            print('Check:      ', m * self.noise_std)
            print('Outlier:    ', dist_from_mean >= m * self.noise_std)
            
        return dist_from_mean >= m * self.noise_std
        #return False

    def is_outlier(self, activation, report=False):
        previous = np.array(self.last_activations) * 1.5
        percentile_75 = np.percentile(previous, 75)
        outlier_p = activation >= percentile_75 and activation > 0

        if report and report == 'cam_0':
            print('\nCamera: ' + report)
            print('  - 75th Percentile: ' + str(percentile_75))
            print('  - Activation: ' + str(activation))
            print('  - IS OUTLIER: ' + str(outlier_p) + '\n')

        return outlier_p
        
        
    def update_threshold(self):
        """Update the threshold by using
        mean + constant * std

        Returns:
            float: new threshold
        """
        return self.mean + self.threshold_constant * self.std
    
    def make_decision_old(self, filt_act, threshold):
        """Make a stopping decision.
        Will return true if filt_act > threshold and
        there all other items in self.decisions are True.

        Args:
            filt_act (float): filtered activation
            threshold (float): current threshold

        Returns:
            bool: stopping decision
        """
        self.decisions.append(filt_act >= threshold)
        # print('\n')
        # print(filt_act)
        # print(threshold)
        # print(self.decisions)
        # print('\n')
        if sum(self.decisions) == self.min_decision:
            return True
        return False

    def make_decision(self, cam=False):
        grads = np.gradient(np.array(self.last_activations))
        increasing = grads[np.where(grads > 0.01)].size
        if cam == 'cam_n45':
            print('\nGradients for camera ' + cam)
            print(grads)
            print('')
        return increasing >= 7

    def step(self, activation, distance=False, cam=False):
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
            if not self.is_outlier(activation, report=False):
                # If it is not, update stats and threshold
                self.mean, self.std = self.update_stats(activation, self.n, 
                                                        self.mean, self.std)
                self.n += 1
                
                threshold = self.update_threshold()
                # Make decision
                decision = self.make_decision(cam=cam)

                if self.report:  # Only relevant for reporting
                    self.report_distance.append(distance)
                    self.report_activations.append(activation)
                    self.report_thresholds.append(threshold)
                    self.report_mean.append(self.mean)
                    if decision:
                        self.report_decisions.append(activation)
                    else:
                        self.report_decisions.append(np.nan)
                    
                return decision, threshold

                
        elif self.started:
            self.n += 1
            # Check whether we can initialise
            self.check_init()
            
            if self.report:  # Only relevant for reporting
                self.report_activations.append(self.mean)
                self.report_thresholds.append(np.nan)
                self.report_mean.append(self.mean)
                self.report_decisions.append(np.nan)
                self.report_distance.append(np.nan)

        thr = self.is_outlier(activation)
        if thr: 
            return False, 'OUTLIER'
        else:
            return False, 0
                    
