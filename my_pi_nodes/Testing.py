from Control import robotmodel

e  = 45   
f  = 80  
re = 272
rf = 235

def test_robotmodel():
    # Create a robot model instance
    model = robotmodel(f,e,re,rf)

    position = model.delta_calcInverse(0, -10, -200)
    print(f"Inverse Kinematics for (0,0,-200): {position}")

if __name__ == "__main__":
    test_robotmodel()