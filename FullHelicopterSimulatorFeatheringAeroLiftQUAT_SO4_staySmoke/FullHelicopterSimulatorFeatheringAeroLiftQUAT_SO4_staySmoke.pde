  // Shawn Daniel    //<>// //<>// //<>// //<>//
  // Code Created: 3/12/2019
  // Note: Backstepping ontroller is used on the altitude controller. Backstepping control is virtual but it closely tracks desired tracking rate of the target signal
        // Using LQR controller/ Feedback Linearization to track states to the backstepped control tracks the final goal position, and the goal velocity of the target signal!!!
  

  
  import peasy.*; 
  import controlP5.*;  
  import oscP5.*; 
  import netP5.*; 
  
 
  float MAX_TRANSLATION = 1;  // To  allow for tranasation of prismatic robotics.
  float MAX_ROTATION = 1;    // for simulating rotation of our object.  In radians
  float  ang2, dang2;
  float ddang1x,ddang1y,ddang1z; // acceleration variable.
  float dt1= .05;//.05;  // dicrete time for simulation dynamics, look at frameRate(*) dt= 0.009 is vary accurate and quite fast.  dt1= 0.01  is fast and it works
  boolean llave, llave2, llave3,llave4,llave5,llave6,llave7,llave8;


  // DYNAMICS PARAMETERS
   float g = 9.81; 
   
   // Masses and Weights
   float mMR = 350;    // [kg]
   float mFuse = 1650 ;  // Fuselage mass.  [kg]
   float mTot= mMR+mFuse ;   // [kg]
   float weight= g*mTot  ;  
   
   // Mass moment arms and Moments of Inertia.
   float lm = 10;  // [m]  10
   float l_fuse = 13;           // [m]  
   float I_MR   = mMR*lm*lm ;   // [kg-m^2]
   float I_Fuse = mFuse*l_fuse*l_fuse ;   // [kg-m^2]
   float I_tot= (I_MR+I_Fuse)*0.1/0.1;  
   float Jx=I_tot/3.0,Jy=I_tot/1.2,Jz=I_tot/20;
   float cntrlRoll=0, cntrlPitch=0; 
   float cntrlYaw=0;


   
   float x= 200*0 ;      // Initialize at a displacement to view loop maneuver.
   float dx = 140*0;
   float ddx = 0; // iniitally at rest.
   float y = 0;
   float dy = 0;
   float ddy = 0;
   float z = 400;
   float dz = 0;
   float ddz = 0;
   float z0;
   float cntrl=0; 
   float cntrlt=0; 
   
   // For SMC 
    float Asmcx=10;
    float Bsmcx=10;
    float manifold=0.0;
    float smcCntrl=0.0;  
    float epsilon= 1;
    
   // Rotational State Variables.
   float MR=0, dMR=0, ddMR=0;  // Make this the rotation of the M.R. and saturate the anguar increase within one wrap around.
   float dMRt=0;
   float ddMRLimit= 4000000;
   float alpha=0, dalpha=0, ddalpha=0;
   float theta=0, dtheta=0, ddtheta=0;
   float psi=0, dpsi=0, ddpsi=0;
   
   // Lift Realization Variables
   float Lift, Liftm1=0, dLift=0;
   float rho_sl=1.225, r_MR=5.969, Cl=3.0746, coordMR=.1016, coordTR=coordMR;
   
   float alphaI= 180*PI/180, dalphaI=0, ddalphaI=0;
   float thetaI=0, dthetaI=0, ddthetaI=0;
   float psiI=0, dpsiI=0, ddpsiI=0;
   
   // Max Flap Deflection
   float maxFlapDeflection = 45.0; // 10 degrees is best.
   
   // trajectory magnitude linear contrl saturation.
   float trajNeighBall=80;

   // Controller Parameters (Gains)
     float k1z = 100.0 ;   // k1 = {100}   
     float k2z = 1000.0 ;  // k2 = 1000.00
     float k0z = 10.0 ;    // k3 = 10.0
     float k1Lat = 1.0 ;  // k1= {0.05}  
     float k2Lat = 2.0 ;
     float k0x = 1.0 ;
     
   // Note: the below control gains NEEEDED to prevent instability in inertial posiitoning fix.
   float k1Bank  =  5.0;  
   float k2Bank  =  20.0;  
   float k1Pitch =  5.0 ;
   float k2Pitch =  20.0;
   float k1AltBank = 0.0 ;
   float k2AltBank = 0.0*5 ;
 
   // Maneuver Section
  float bankCmd=0, pitchCmd=0;
  float bankLoop=0;  // This will be updated to perform the loop maneuver.
  float EnergyLoop=0;
  float Vx=0;  // These will also be updated, in trajectory detection in real time.
  float Vz=0;
  float Vmag=0;
  float gComp=0;
  float rCurvature=0;
    
   // More control gains
   float Gp1=20*1.5, Gp2=20*1.5, Gp3=20*1.5;
   float Gr1=10*3, Gr2=10*3, Gr3=10*3; 
   float gamma=20;
     
    // Quaternion Logic
   float[] quat;
    
   float aeroFrictBank  = 0.8;  // For  errounously large dt1, we need significant friction on the A/C to dampen the rotational acceleration.
   float aeroFrictPitch = 0.8;
   
   // Control Regime Saturation.
   float satur = 0.05 ;  // Saturated value to create local stability region.
                           // sat= {0.1, 0.13, 0.05}   0.1 for LQR and 0.13 for SMC 0.05 for SwingUp.
   float satur2= 2.0  ;
   float satur3= 6.5;
 
 // Friction
   float frict= 3;
 
   float ke=1;
   float b0x =1, b0y =0, b0z =0; // I must extract the current euquat and do pitch rotation on it " in the ontroller" to determine the desired pit
   float b0norm= 0;

   float smc=0;
   
   // Tail Component Variables.
   float Liftt;  // Tail rotor lift.
   float tailBoom= 6.096; // [m]
   float Clt=.120,  r_TR=5.969;
    
  ControlP5 cp5;
  PeasyCam camera; 
  Pendulum mPendulum;    
  OscP5 oscP5;
  NetAddress mOscOut;
  
  float posX=0, posY=0, posZ=0, rotX=0, rotY=0, rotZ=0, refAlt=0; 
  float refX=0, refY=0, drefX=0;
  float ix, jy, kz;  
  float qx, qy, qz;
  float q0x, q0y, q0z;
  float A1=0, B1=0, theta0=0, P1=0;
  float feather=0;
  float cntrlx, cntrly, cntrlz; 
  boolean ctlPressed = false;
 
 
  // Dust Trail
    import java.util.*;
  LinkedList<PointXY> trail;
  int maxTrailLength = 400; // */
  
  
  void setup() {
    // Choose a managable window size to plot simulation.
     size(768, 639, P3D); // Form 3D window.
   
    smooth();
    frameRate((int)(1/dt1)); 
    textSize(20);
    mOscOut = new NetAddress("192.168.0.24", 8888);
    camera = new PeasyCam(this, 650); 
    camera.setRotations(-1.0, 0.0, 0.0); 
    camera.lookAt(8.0, -50.0, 80.0); 
    mPendulum = new Pendulum(1); 
    mPendulum.applyTranslationAndRotation(new PVector(), new PVector(), new PVector(), new PVector(), new PVector(), new PVector(), new PVector(), new PVector());  
    cp5 = new ControlP5(this); // Create control object.
    cp5.addSlider("refAlt").setPosition(width-210, 370).setSize(180, 40).setRange(0, 300);
    cp5.addSlider("refX").setPosition(width-210, 420).setSize(180, 40).setRange(-150, 150);
    cp5.addSlider("refY").setPosition(width-210, 490).setSize(180, 40).setRange(-150, 150);
    // Maneuver Section
    cp5.addSlider("bankCmd").setPosition(width-210, 530).setSize(180, 40).setRange(-180, 180);    
    cp5.addSlider("pitchCmd").setPosition(width-210, 560).setSize(180, 40).setRange(-180, 180); 
    cp5.addSlider("yawCmd").setPosition(width-210, 590).setSize(180, 40).setRange(-180, 180); 
    cp5.setAutoDraw(false); // B/c auto draw places  image of the control pannel in location in the xy-plane. We want to place it ourselve.
    camera.setActive(true); // Not sure of this ones meaning.
    
    // Dynamics Simulation.
    llave= false; // For triggering the dynamic laws on the system.
    llave2= false; // For triggering control system.
    llave3= false; // For triggering Full State Feedback.
    llave4= false; // For triggering perturbations.
    
    trail = new LinkedList<PointXY>();
    stroke(249, 233, 7);
    strokeWeight(1);
    
} // End of setup


public class PointXY {
  public float x, y, z;
  public PointXY(float px, float py, float pz) {
    x = px;
    y = py;
    z = pz;
  }
} 
 
  void draw() {  
    background(200); 
    mPendulum.applyTranslationAndRotation(PVector.mult(new PVector(posX, posY, posZ), MAX_TRANSLATION), 
      PVector.mult(new PVector(rotX, rotY, rotZ), MAX_ROTATION), 
      PVector.mult(new PVector(A1, B1, theta0), 1),
      PVector.mult(new PVector(feather,P1,MR ), 1),
      PVector.mult(new PVector(b0x,b0y,b0z), 1),    
      PVector.mult(new PVector(dx, dy, dz),1),
      PVector.mult(new PVector(refX-x, refY-y, refAlt-z),1),
      PVector.mult(new PVector(0, cntrlPitch, cntrlYaw),1)); // feather does the feathering actuation.
           // This should be the desired placement minus the current placement.
    PVector erqrefqk = mPendulum.getErQuatError();  // Here extract our targeting quaternion error.      
           
    PVector desOmDot = mPendulum.getDesOmegaDot();       
       
    mPendulum.draw();
  
    hint(DISABLE_DEPTH_TEST); 
    
    camera.beginHUD(); //?
   
    cp5.draw(); 
    camera.endHUD();
    
    hint(ENABLE_DEPTH_TEST); // Play with this.
  
  // Populate Smoke trail of the A/C
  if (trail.size() >= 2) 
  {
    PointXY currPoint, lastPoint = trail.get(0);
    for (int i = 0; i < trail.size(); i++) 
    {
      currPoint = trail.get(i);
      //stroke(249, 233, 7);
      strokeWeight(1);
      line(lastPoint.x, lastPoint.y, lastPoint.z, currPoint.x, currPoint.y, currPoint.z);
      lastPoint = currPoint;
    }
  }
  
  PointXY p = new PointXY(x, y, z); // call in values into the new linked list, can scope here see these variables?
  trail.addFirst(p);

  fill(200);

   
  // BELOW TURNS SIMUNATION ENVIRONMENT ON IF KEYS AND SWITCHED.
  if(llave){  // Apply dynamic laws on the system. Activate Dynamics.
   
   // EOM OF POSITION ( In Inertial frame.)
   // Bon
    posX = x;
    posY = y;
    posZ = z;
    
    // Bon
    rotX = alphaI; // negetive to make sime look good, do to frame transformations.
    rotY = thetaI;
    rotZ = psiI;
    
   // Note: Add fuselage drag
    ddx = -1.0 / mTot *(cos(-psiI)*sin(thetaI)*cos(alphaI)+sin(-psiI)*sin(alphaI)) * (Lift*cos(B1*PI/180.0));  // body frame cos(B1*PI/180.0)
    ddy =  1.0 / mTot *-(sin(-psiI)*sin(thetaI)*cos(alphaI)-cos(-psiI)*sin(alphaI)) * (Lift*cos(A1*PI/180.0));     // body frame
    ddz =  1.0 / mTot *( Lift*cos(A1*PI/180.0)*cos(B1*PI/180.0)*cos(alphaI)*cos(thetaI)-weight); // Inertial frame
   
   // EOM OF ROTATION ( In the body frame)
     // N ote that here the control is the U1('Omega_M.R.'), A1 and B1, and P1.
   ddalpha= 1/Jx * ( Lift*sin(A1*PI/180.0)*lm ) + 1/Jx*dtheta*dpsi*(Jz-Jy) - dalpha * aeroFrictBank*0.0  ;   // Lessen the magntude of the weight on the bank control. This should let rotation occur easier.
   ddtheta= 1/Jy * ( Lift*sin(B1*PI/180.0)*lm ) + 1/Jy*dalpha*dpsi*(Jx-Jz) - dtheta * aeroFrictPitch*0.0 ;
   ddpsi  = 1.0/Jz * (Liftt*tailBoom) + 1/Jz*dtheta*dalpha*(Jy-Jx) - 1*ddMR/1000; // */
   
    
    // MARKOV Update. 
    alphaI = alphaI +dalphaI*dt1; 
    dalpha += ddalpha * dt1 ;   
    dalphaI = dalpha + dtheta*sin(alphaI)*tan(thetaI) - dpsi*cos(alphaI)*tan(thetaI) ; 
    thetaI  = thetaI+ dthetaI*dt1;
    dtheta += ddtheta * dt1;   
    dthetaI = dtheta*cos(alphaI) + dpsi*sin(alphaI); 
    psiI  = psiI + dpsiI*dt1; 
    dpsi += ddpsi * dt1;     
    dpsiI = (-dtheta*sin(alphaI)* 1/cos(thetaI) + dpsi*cos(alphaI)*1/cos(thetaI))*1; 
    
    // Saturate the inertial bank, pitch and yaw.
    if(alphaI > PI){ 
        alphaI = alphaI - 2 * PI;
     }
     if(alphaI <= -PI){
        alphaI = alphaI + 2 * PI;
     }  
    if(thetaI > 2*PI){
        thetaI = thetaI - 2 * PI;
     }
     if(thetaI <= 0){
        thetaI = thetaI + 2 * PI;
     }
     if(psiI > 2*PI){
        psiI = psiI - 2 * PI;
     }
     if(psiI <= 0){
        psiI = psiI + 2 * PI;
     }
    Liftm1=Lift;
  
   // EOM OF M.R.
    MR   = MR + dMR*dt1;
    dMR += ddMR*dt1;
    // FLIGHT MODE.
    ddMR = 20.0 - .0003*dLift;
    dMRt = 60*dMR;
    // dMR and ddMR SATURATE.
    if(dMR > 37.17551)
    {
      dMR =37.17551;
    }
    if(dMR < 0)
    {
      dMR =0;
    }
    if(ddMR > 4000000)
    {
      ddMR =4000000;
    }
    if(ddMR < -4000000)
    {
      ddMR =-4000000;
    }  // */
  
   // Lift Equation Calculation.
     Lift = 4*.5*rho_sl*(dMR*dMR*r_MR*r_MR)*Cl*(theta0)*coordMR*r_MR;
     Liftt= 4*.5*rho_sl*(dMRt*dMRt*r_TR*r_TR)*Clt*(P1)*coordMR*r_TR;
     dLift= (Lift-Liftm1)/dt1;
     
    // EOM and A/C boundary constraints applied below.
     if(z<0)
     {  // This is the air feild or ground
       z = 0;
       ddz = 0;
       dz = 0;
       // Note: if the Helicopter hits the ground boundary that the Heli simulation will glitch really bad and move super fast everywhere!
       ddx=0;
       ddy=0;
       dx= 0;
       dy=0;
     }
     
      stroke(255);  
 //<>// //<>//
         if (MR*180/PI>3600) {
           MR= MR-20.0*PI;
         }  

        // POSITION MARKOVIAN CALCULATION. //<>// //<>// //<>// //<>//
        x = x + dx*dt1;   
        dx += ddx*dt1;      
        
        y = y + dy*dt1;   
        dy += ddy*dt1;     
       
        z = z + dz*dt1;   
        dz += ddz*dt1;      
        z0 += (z-refAlt)*dt1;  // Here we take integral term of the variable we're rying to regulate to. Must fight Gravity in our controller.
          
        qx= erqrefqk.x; 
        qy= erqrefqk.y;
        qz= erqrefqk.z;
    }
  
      if (llave2 == false){  
        cntrl=0; //     Turn lift off.
      }
      
    if(llave3 & llave2){    // Turn on control  Full state Feedback.
       
       float z1= (refAlt-z);
       float z2= dz - 0 - 1*z1;

       if(cos(alphaI)>cos(80*PI/180) || cos(thetaI)> cos(80*PI/180)){ // Safe operationg regime.
            //Backstepping control.
            cntrl = mTot /(cos(alphaI)*cos(thetaI))* (z1 + g - 1*z2 - 1*(z2+ 1*z1) ); 
            theta0=cntrl/(4*.5*rho_sl*(dMR*dMR*r_MR*r_MR)*Cl*coordMR*r_MR); 
            if(theta0>15*PI/180){
                theta0=15*PI/180; 
             }
             if(theta0<-0*PI/180){
                theta0=-0*PI/180; 
             }   // */
             
       }
       if (cos(alphaI)<cos(80*PI/180) || cos(thetaI)< cos(80*PI/180)) {   // Unsafe regime.
             theta0=5*PI/180; // One degree of difflection.  Five degrees is good for reorientation.
           // Power off version for the backstepping control.
           //ddMR=-dMR*.008;
       }
      
       // Below is the cyclic control 
       // Extract body frame trajectorey error.
       PVector trajError= mPendulum.getBodyFrameTrajError();
       PVector velBodyFrame= mPendulum.getBodyFrameVelError();
       
       // Saturate control error to prevent (overactuating) flipping over the A/C
      float ydisErr=-trajError.y; float xdisErr=trajError.x;
       if(xdisErr> trajNeighBall){
         xdisErr = trajNeighBall;
       }
       if(xdisErr< -trajNeighBall){
         xdisErr = -trajNeighBall;
       }
       if(ydisErr> trajNeighBall){
         ydisErr = trajNeighBall;  
       }
       if(ydisErr< -trajNeighBall){
         ydisErr = -trajNeighBall;
       } // */
        
       cntrlRoll   = - (k1Lat*(ydisErr)+ k2Lat*3*(velBodyFrame.y) + k1Bank*(alphaI*180/PI-bankCmd)+ k2Bank*(dalphaI-0.0) ); 
       cntrlPitch  = - (k1Lat*(xdisErr) + -k2Lat*3*(velBodyFrame.x) + k1Bank*(-0)+ k2Bank*(0.0-0.0));  
       
       // Limit cntrl pitch so I dont pitch A/C too much!
       if(cntrlPitch>10){
         cntrlPitch=10;
       }
       if(cntrlPitch<-10){
         cntrlPitch=-10;
       }
       if(cntrlRoll>30){
         cntrlRoll=30; //<>// //<>//
       }
       if(cntrlRoll<-30){
         cntrlRoll=-30; 
       } 
       
       if(abs(trajError.y)+abs(trajError.x)> 30){
         cntrlYaw= atan2(y-refY,refX-x);
       }
       
       // Here allows the desired commanding of our guiding  quaternion.
       b0x = cos(cntrlPitch*PI/180)*cos(-cntrlYaw);   
       b0y = cos(cntrlPitch*PI/180)*sin(-cntrlYaw);   
       b0z = sin(cntrlPitch*PI/180);
       //*/
      
       b0norm= sqrt(b0x*b0x + b0y*b0y + b0z*b0z); //<>// //<>//
       if(cntrl> 2*weight){
         cntrl = 2*weight-100; // good reset to prevent setting sticking
       }
       
       smc = 19.9*2; 
       float dsmc= 2.50*5.1; 
       
       //   QUATERNION FIXING CONTROL.
         cntrlx= -0.5*( (Gp1*q0x+gamma*(1-q0x))*qx + -qy*qz*Gp2 + qy*qz*Gp3 ) - Gr1*dalphaI  + -smc*(alphaI-cntrlRoll*PI/180) + -dsmc*(dalphaI); 
         cntrly= -0.5*( Gp1*qx*qz + (q0x*Gp2+gamma*(1-q0x))*qy + -qx*qz*Gp3 ) - Gr2*dtheta + 1*desOmDot.y;  // The latter is feed forward compensator.
         cntrlz= -0.5*( -Gp1*qx*qy + qx*qy*Gp2 + (q0x*Gp3+gamma*(1-q0x))*qz ) - Gr3*dpsi +-1*desOmDot.z*0;  // */   
       // Final realization.
       
       // Calculate the P1 neccesary to achieive the desired control value.
       A1= cntrlx;  // cntrlx rolls about desired quaternion
       B1= cntrly;  // cntrly pitches the desired quaternion
       P1= cntrlz/( 4*.5*rho_sl*(dMR*dMR*r_TR*r_TR)*Clt*coordTR*r_TR*tailBoom); // */  // I want this in radians!
      
      //Saturate the P1 angle.
      if (P1>15*PI/180){
        P1= 15*PI/180;
      }
      if (P1<-15*PI/180){
        P1= -15*PI/180; //<>// //<>//
      } 
      
       // Control Actuator Limits
        if (A1>maxFlapDeflection) {    // Note cyclic commands are in degrees.
       A1= maxFlapDeflection;
       }
       if (A1<-maxFlapDeflection) {    // Note cyclic commands are in degrees.
       A1=-maxFlapDeflection;
       }
       if (B1>maxFlapDeflection) {    // Note cyclic commands are in degrees.
       B1=maxFlapDeflection;
       }
       if (B1<-maxFlapDeflection) {    // Note cyclic commands are in degrees.
       B1=-maxFlapDeflection; //<>// //<>// //<>//
       } // */
       
       
    }
    
      // Below is control activation key, for new control algorithms
      if (llave4 == false){  
        loop();
      } else {
        noLoop();
      }
      
   
  } // End of void draw, the continuous real time simulation loop.
  
  

  
  void controlEvent(ControlEvent theEvent) {
    camera.setActive(false); 
  }
  
  void mouseReleased() {
    camera.setActive(true); 
  }
  
  long lastTime = 0; // Time stamp.
  
 
  void mouseDragged ()   // With click on.
  {   
  
    if (ctlPressed) {    //if (true) then...
      posX = map(mouseX, 0, width, -1, 1); // Set commanded planar Robot translation. Yes.                                        
      posY = map(mouseY, 0, height, -1, 1);
      // Note: (u,v) coordinates of Mouse scaled between -1 and 1.
      // Note: here we set planar position of platform without using the slider controls
    }
  }
  
  
  // Analyse here.
  void keyPressed() {    // This code runs if I press space bar. This resets viewing camera on the 3D object
    if (key == ' ') {
      camera.setRotations(-1.0, 0.0, 0.0);  
      camera.lookAt(8.0, -50.0, 80.0);   
      camera.setDistance(650);      
    } else if (keyCode == CONTROL) {  
      camera.setActive(false);  
      ctlPressed = true;
      llave=  false;  
      llave2= false;
      llave3= false;
      llave4= false;
      llave5= false;
      llave6= false;
      llave7= false;
      llave8= false;
    }
    
  // BELOW WE SWITCH CERTAIN SIMULATION MODES ON AND OFF.
    else if (key == 'g'){  
      llave=true;        
      } 
      else if (key == 'h'){
       llave2=true; 
      }
       else if (key == 'j'){
       llave3=true; 
      }
        else if (key == 'k'){
       llave4=true; 
      }
        else if (key == 'l'){
       llave5=true; 
      }
        else if (key == 'm'){
       llave6=true; 
      }
        else if (key == 'n'){
       llave7=true; 
      }
        else if (key == 'b'){
       llave8=true; 
      }
      else if (key == 'G'){
       llave=false; 
      }
      else if (key == 'H'){
       llave2=false; 
      }
      else if (key == 'J'){
       llave3=false; 
      }
      else if (key == 'K'){
       llave4=false; 
      }
      else if (key == 'L'){
       llave5=false; 
      }
      else if (key == 'M'){
       llave6=false; 
      }
      else if (key == 'N'){
       llave7=false; 
      }
      else if (key == 'B'){
       llave8=false; 
      }
  }
  
  void keyReleased() {   // Here as a redundancy. It works.
    if (keyCode == CONTROL) {
      camera.setActive(true);
      ctlPressed = false;
    }
  }
  
  
  
  
  
  
  
  
  
