## CrazyClover-Experiment

### This project contains
- Crazyflieベースの中型ドローンを利用した実験

### Dependencies
- numpy

### Experiment
- hovering.py : ホバリング実験
- circle_tracking.py : 円軌道追従実験 ( sin and cos function )
- polytraj_tracking.py : 直線軌道追従 ( polynominal function )

#### circle trajectory tracking
```python
python3 circle_tracking.py
```
https://user-images.githubusercontent.com/64090003/200737571-99d39ad9-0197-49a2-a5ca-0e7031bd1def.mp4

### references 
- [Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag for Accurate Tracking of High-Speed Trajectories - Matthias Faessler, Antonio Franchi and, Davide Scaramuzza](https://arxiv.org/pdf/1712.02402.pdf)
- [Minimum Snap Trajectory Generation and Control for Quadrotors - Daniel Mellinger and Vijay Kumar](https://arxiv.org/pdf/1706.06478.pdf)
- [Polynomial Trajectory Planning for Aggressive Quadrotor Flight in Dense Indoor Environments - Charles Richter, Adam Bry, and Nicholas Roy](https://groups.csail.mit.edu/rrg/papers/Richter_ISRR13.pdf)
