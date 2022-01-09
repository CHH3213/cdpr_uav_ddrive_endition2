```python
x_p,x_i,x_d = 1,1,0.5
y_p,y_i,y_d = 1,1,0.5
z_p,z_i,z_d = 1,1,0.5
x, y, z = 5, 5, 30
self.control_x = PID(np.asarray( [0.15*x_p, 0.0001*x_i, 0.025*x_d])*x, target[0], upper=250, lower=-250)  # control position x
self.control_y = PID(np.asarray( [0.15*y_p, 0.0001*y_i, 0.025*y_d])*y, target[1], upper=250, lower=-250)  # control position y
self.control_z = PID(np.asarray([0.03*z_p, 0.0002*z_i, 0.3*z_d])*z, target[2], upper=100, lower=-100)  # control position z

```

PID参数为：

- 

  |           | P    | I      | D      |
  | --------- | ---- | ------ | ------ |
  | x方向控制 | 0.75 | 0.0005 | 0.0625 |
  | y方向控制 | 0.75 | 0.0005 | 0.0625 |
  | z方向控制 | 0.9  | 0.006  | 4.5    |

  

