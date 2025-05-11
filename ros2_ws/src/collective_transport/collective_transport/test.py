
import numpy as np
err_theta =  315
if err_theta < -180 or err_theta > 180:
    print("bitch")
    err_theta = np.sign(err_theta) * (-1) * (err_theta % 180) # tell it to go other way, "it's closer the other way"
    
print(err_theta)