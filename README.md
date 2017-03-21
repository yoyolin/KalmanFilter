# KalmanFilter




## Introduction

Kalman Filter, an artificial intelligence technology, has been widely applied in driverless car navigation and robotics. It works well in presence of uncertainty information in dynamic systems. It is a linear solution based on Bayesian Inference especially for state space models. -- This is the definition in the hard way.



Lets skip the first paragraph and look at a little story (story source: [Kalman Filter: Theories and Applications](http://blog.csdn.net/heyijia0327/article/details/17487467))

A group of young men were standing, under their feet there was a twisty narrow road to a very big tree. One man walked to the tree, asking, "can any of you walk to the tree with your eyes closed?" 

"That's simple, I used to serve in the army." Mike said. He closed his eyes and walked to the tree like a drunk man. "Well, maybe I haven't practiced for long." He murmured. <font color="red"> - Depending on prediction power alone.</font>

"Hey, I have GPS!!" David said. He held the GPS and closed his eyes, but he also walked like a drunk man. "That's very bad GPS!" He shouted, "it's not accurate!!" <font color ="red"> - Depending on measurement which has big noises</font>

"Let me try." Simon, who also served at the army before, grabbed the GPS and then walked to the tree with his eyes closed. <font color="red"> - Depending on both of prediction power and measurement </font>

After reaching the tree, he smiled and told everyone, "I am Kalman."






In the story above, a good representation of the walking **state ** at time k is the velocity and position.

$$ X_k  =  (p, v) $$

So there are two ways to measure where you are. 

- you **predict** based on your own command system - it records every commands sent to you, but only some of commands are executed exactly as what they were - wheels may slip or wind may affect;
- you **measure** by your GPS system - it measures where you are based on satellite, but it can't be as accurate as in meters and sometimes signals lost.



With Kalman Filter, we will get better understanding of where you are and how fast you go than either of the prediction or measurement. That is, we update our **belief** of where you are and how fast you go by incorporating the two sources of **predictions** and **measurements** using Bayesian inference. 



## Understanding Kalman Filter

Note: all the knowledge and photos for this section come from [Ref 2](http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/). This is a study note only.



The whole idea of Kalman Filter can be represented by a single picture.  It might look complicated at this moment, but we will understand everything after this article (if not, read  [Ref 2](http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/) - it's a much nicer article I believe).

![kalman filter](http://www.bzarg.com/wp-content/uploads/2015/08/kalflow.png)

[source: ref 2]

### Prediction Phase: $X_k, P_k, F_k$

#### $X_k$: Position and Velocity

Remember in our scenario, we want to know the position and velocity.  We represent the **state** of the walking people at time k as $X_k = [position_k; velocity_k]$. 

#### $F_k$: Prediction Matrix

We have

$$ Position_k =  Velocity_{k-1}*t + Position_{k-1} $$ 

$$ Velocity_k =            Velocity_{k-1}$$



The above two formulas could be written as:

$$ [\begin{matrix} Position_k \\ Velocity_k \end{matrix}] =  [\begin{matrix} 1 & t \\ 0 & 1 \end{matrix}]* [\begin{matrix} Position_{k-1} \\ Velocity_{k-1} \end{matrix}]  $$

We represent the prediction matrix $ [\begin{matrix} 1 & t \\ 0 & 1 \end{matrix}] $ as $F_k$. We have then $X_k = F_k * X_{k-1}$. 

#### $P_k$: Covariance Matrix

Since in our case, the faster the robot walks, the further the position might be. Therefore velocity and position should be correlated; the covariance matrix is $P_k$.

When prediction matrix updates X_k, the change will also reflect on covariance matrix. Therefore $P_k = F_k * P_{k-1} * F_k^T$.



### Measurement Phase: $Z_k, H_k$

#### $H_k$: Measurement Function

Sometimes the GPS reading is not having the same units with the prediction states. For example, we use km/h in prediction states and we use m/s in GPS reading for velocity. So we need a transformation with matrix $H_k$: 

$$ X_k = H_k * X_k $$

$$ P_k = H_k * P_k*H_k^T​$$



#### $Z_k$: Measurement

Remember the GPS reading is not very reliable and might have some variations. So it is represented as a Gaussian distribution with **mean** $Z_k =  [PositionReading_k, VelocityReading_k]$. So in below picture the pink circle represents the prediction distribution while the green circle represents the measurement distribution. The bright bubble insight represents the belief distribution of position and velocity.

![bayesian](http://www.bzarg.com/wp-content/uploads/2015/08/gauss_6a.png)

[source: ref 2]

### External Factors: $R_k, Q_k, B_k, U_k $

#### $R_k, Q_k$: noises

$Q_k$ is the transition covariance for the prediction phase. $P_k = P_{k-1} + Q_k$. The idea is that there would always be some uncertainty. Therefore we kept an assumption - points might move a bit outside its original path. In practice, the value is often set very slow, for example: 

```
delta = 1e-5
Q = np.array([delta/1-delta, 0; 0, dealta/1-delta])
```

$R_k$ is the observation covariance for the measurement phase. $P'_k = P'_{k-1} + R_k$. Default set as 1.

#### $B_k, U_k$: External influence

For example, if the people are going down from the mountain, he might walker quicker because of the gravity. Therefore we have

$$ Position_k =  Velocity_{k-1}*t + Position_{k-1} + 1/2*g*t^2 $$ 

$$ Velocity_k = Velocity_{k-1} + g*t $$

Therefore we have

$$X_k = F_k * X_{k-1} + [\begin{matrix} t^2/2 \\ t \end{matrix}]*g = F_k*X_{k-1} + B_k*U_k$$

where $B_k$ is the control matrix for external influence and $U_k$ is the control vector. 

### The Big Boss: Kalman Gain ?

We are almost done explaining all the variables in Kalman Filter, except a very important term: Kalman Gain. This is a bit complicated, but luckily, this is not something we need to calculate or input.

Now let's go back to the measurement phase once more. When we multiplying the prediction distribution and measurement distribution, the new mean and variance go like this:

$$ u' = u_0 + \sigma_0^2 (u_1 - u_0)/ (\sigma_0^2 + \sigma_1^2)$$

$$ \sigma'^2 = \sigma_0^2 - \sigma_0^4/(\sigma_0^2 + \sigma_1^2) $$

so we make: $k = \sigma_0^2/ (\sigma_0^2 + \sigma_1^2) $ where k is the kalman gain, therefore we can simplify the above equations to:

$$u' = u_0 + k*(u_1 - u_0)$$

$$ \sigma'^2 = (1-k)\sigma_0^2$$

Therefore Kalman gain helps updating the new $X_k, P_k$ value after seeing the measurement. So what is an intuitive explanation of Kalman Gain? It actually calculates the uncertainty in the prediction phase to the measurement phase, so it tells how much we should trust the measurement when updating $X_k, P_k$. 

## Python Implementation

I'd love to recommend [a great post](http://www.thealgoengineer.com/2014/online_linear_regression_kalman_filter/) which gives applications of Kalman Filter in financial predictions with codes posted on its [Jupyter Notebook](http://nbviewer.jupyter.org/github/aidoom/aidoom.github.io/blob/master/notebooks/2014-08-13-online_linear_regression_kalman_filter.ipynb). It demonstrates why we should use Kalman Filter comparing to linear regression just in one picture:

![kalman application](http://www.thealgoengineer.com/img/2014-08-13-online_linear_regression_kalman_filter/price_corr.png)[source: ref 3]

### Parameter Mapping

Recall: Kalman Filter measures **uncertain information** in a **dynamic systems**. In this case, we want to know the hidden state slope and intercept. Let's map all the inputs from theoretical to practical settings.

#### Prediction Phase

* State: $$X_k =[\begin{matrix} \alpha_k \\ \beta_k \end{matrix}]$$
* Prediction Matrix: $F_k = [\begin{matrix} 1&0\\0&1 \end{matrix}]$. <br>This is because we assumes the slope and intercept aren't correlated.
* Intial State Covariance $P_0 = [\begin{matrix} 1 & 1 \\ 1 & 1 \end{matrix}] $

#### Measurement Phase

* Measurement Function $H_k = [\begin{matrix}EWA & 1 \end{matrix}]$. <br> The measurement we have is EWC. It's obvious that it doesn't share the same measuring units with slope and intercept. Since we have EWC = EWA*slope + intercept, therefore the measurement function should be [EWA 1].
* Measurement Mean $Z_k = EWC$. 

#### External Factors

* Transition Covariance $Q = [\begin{matrix} 1e-5/ (1-1e-5) & 0 \\ 0 1e-5/(1-1e-5) \end{matrix}] $
* Observation Covariance R =  1

Note that, the selection of Q and R here means the author wants to trust more in the prediction phase rather than the measurement phase.

Or, as suggested in PyKalman documentation, values for $P_0, Q, R$ could be initialized using:

```
 kf = KalmanFilter(em_vars=['initial_state_covariance', 'transition_covariance', 'observation_covariance'])
 kf.em(X, n_iter=5)
```



### Implementation Codes

Here is the [code source](http://nbviewer.jupyter.org/github/aidoom/aidoom.github.io/blob/master/notebooks/2014-08-13-online_linear_regression_kalman_filter.ipynb). I copied it here only for easy reading.

```
import numpy as np
from pykalman import KalmanFilter

### construct the covariance matrix Q and measurement function H
delta = 1e-5
trans_cov = delta / (1 - delta) * np.eye(2)
obs_mat = np.vstack([data.EWA, np.ones(data.EWA.shape)]).T[:, np.newaxis]

### construct Kalman Filter
kf = KalmanFilter(n_dim_obs=1, n_dim_state=2,
                  initial_state_mean=np.zeros(2),
                  initial_state_covariance=np.ones((2, 2)),
                  transition_matrices=np.eye(2),
                  observation_matrices=obs_mat,
                  observation_covariance=1.0,
                  transition_covariance=trans_cov)

### get results
state_means, _ = kf.filter(data.EWC.values)
slope = state_means[:, 0]
intercept = state_means[:,1]
```



## Application in Dynamic RoI

Similarly, diminishing marketing RoI could be measured in this way. We always write 

$$ Sales = Marketing Investment * RoI + Intercept $$

However, as time past by, the RoI should also be diminishing. So with Kalman Filter, the changing RoI could be captured. Meanwhile, Intercept also composite of historical sales, industry trends, buzz news etc and could be analyzed deeper.



## Reference

* [Kalman Filter: Theories and Applications](http://blog.csdn.net/heyijia0327/article/details/17487467)
* [How a Kalman Filter works, in pictures](http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)
* [Online Linear Regression using a Kalman Filter](http://www.thealgoengineer.com/2014/online_linear_regression_kalman_filter/)
* Estimating the Half Life of Advertisement, *Prasad Naik*, 1999
* [How to understand Kalman Gain intuitively](http://dsp.stackexchange.com/questions/2347/how-to-understand-kalman-gain-intuitively)
* [pykalman documentation](https://pykalman.github.io/#optimizing-parameters)

