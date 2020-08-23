# MCL-standalone
Small standalone code to perform MCL localisation 

to run the code 

```
    chmod +x start_MCL.py
    ./start_MCL.py
```
### Pre-requisites 
   - python3
   - numpy 
   - matplotlib
   
## Explanation

  - Robot class is used to model sensor model and motion model (optim_mcl.py)
  - MCL class handles sampling, importance weight, motion update
  - Robot uses simple odomety model instead of velocity motion model. It turns first 
  and then translates. motion command is of the form [turn, forward]
  - Sensor is modelled only with a gaussian dropout 
  - orientation is not explicity used in measurement update 
  - Robot motion inaccuracy is modelled as ```turn_noise, forward_noise```. And measurement 
  inaccuracy is modelled by ```sense_noise``` which is set using
  ``` robot.set_noise(turn_noise, forward_noise, sense_noise)```
  - based on Airobotics course by S.Thrun 
  
# Results
[localisation]('./gifs/MCL-final.gif')
