import numpy as np
def adjust_radian(rad):
    # Ensure radian value is within the range of 0 to 2π
    adjusted_rad =  (rad + np.pi)%(2 * np.pi)- np.pi
    return adjusted_rad
def adjust_v(vl, vr):
    max_v = 0.8
    if vl > 0.8 or vr > 0.8:
        # Calculate the reduction factor
        reduction_factor = max(vl,vr) - max_v
        # Reduce both pwml and pwmr by the same amount
        vl -= reduction_factor
        vr -= reduction_factor
    if vl < -0.8 or vr < -0.8:
        reduction_factor = -0.8 - min(vl,vr)
        vl += reduction_factor 
        vr += reduction_factor 
    return vl, vr
def control(x,y,theta, x_d,y_d) :
    #v = 0.3
    dx = x_d - x
    dy = y_d - y
    theta_adjust = adjust_radian(theta)
    
    # Calculate the angle using arctangent
    phi = np.arctan2(dy, dx)
    deltaPhi = phi-theta_adjust 

    deltaPhi = deltaPhi*5
    print("GOC",deltaPhi)
    # vl = v*(1 + 0.5*deltaPhi)
    # vr = v*(1 - 1.5*deltaPhi)
    v = np.sqrt(dx**2 + dy**2)
    # v= 40*(v/np.sqrt(v))
    pwmr = (2*v + deltaPhi*0.3)/2*0.085
    pwml = (2*v - deltaPhi*0.3)/2*0.085
    k=np.sqrt(pwml**2+pwmr**2)
    ratio1 = abs(pwml)/abs(pwmr)
    ratio2 = abs(pwmr)/abs(pwml)

    pwmr = pwmr/k*255
    pwml = pwml/k*255

    if abs(pwml)<135:
        pwml = np.sign(pwml)*135
    if abs(pwmr)<135:
        pwmr = np.sign(pwmr)*135
    # if abs(pwmr) < abs(pwml):
    #     if abs(pwmr)<135:
    #         pwmr = 135
    #         pwml = np.sign(pwml)*135*ratio1
    # else:
    #     if abs(pwml)<135:
    #         pwml = 135
    #         pwmr = np.sign(pwmr)*135*ratio2

    #vl,vr = adjust_v(vl,vr)
    dirl = 1
    dirr = 1
    # pwml= int(np.abs(pwml)/( k/(200 - 130))) + 130 
    # pwmr = int(np.abs(pwmr)/(k/(200 - 130))) + 130  
    # a = np.sqrt(pwml**2+pwmr**2)
    # pwml = pwml/a*255
    # pwmr = pwmr/a*255
    # print(a)
    # if pmwl < 0 :
    #     pwml = 255 - pwml
    #     dirl = 0 
    # if vr < 0   : 
    #     pwmr = 255- pwmr 
    #     dirr = 0 
    return  int(pwmr), int(pwml), dirr, dirl
    
