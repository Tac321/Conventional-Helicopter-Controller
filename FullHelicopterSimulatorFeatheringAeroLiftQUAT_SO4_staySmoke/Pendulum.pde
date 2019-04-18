// Shawn Daniel  //<>//

class Pendulum {
  private PVector translation, rotation, controller, controller2;  
  private PVector[] baseJoint, bladeJoint, q; //  Yes.
  private float baseRadius, bladeRadius, hornLength, legLength;  // Yes
  private float cg_displ, fuse_displ;
  private float fuse_mass_radius;
  private PVector erquatErr;
  
  private PVector euquat,eurwquat, eumrquat; // last term is the Euler quaternion.
  private PVector erquat,errwquat, ermrquat;
  private PVector b0, brw0, bmr0, uu, uuu;
  private PVector b0m1;
  private PVector b0p, b0v; 
  private PVector globalPitch;

  private float yawErr=0;
  private PVector bodyFrameTrajError, bodyFrameVelError;
  private PVector quatManeuv;
  
  private PVector desOm,desOmm1,desOmDot ;
  
  private float key1= 0;
  private float key2= 0;

  // REAL ANGLES
  
  //
  private final float baseAngles[] = {   
   314.9, 345.1, 74.9, 105.1, 194.9, 225.1 }; 

  private final float bladeAngles[]  = {   
   0.0, 90.0, 180.0, 270.0}; 
   
   private final float tailAngles[] ={
    0.0, 90.0, 180.0, 270.0};
   
  
  // Dynamics parameters.
  private final float sysHeight= 10;
  private final float sl1 = 50;      //  Beam length [m]
  private final float slc1 = sl1/2 ;    // pendulum arm cg [m]
  private final float sm1 = 0.02;       // mass of pendulum arm [kg]
  private final float sm2 = 0.300;      // mass of reaction wheel [kg]
  private final float sI1 = 47*10e-6 ;  // moment of Inertia of pendulum arm [kg. m^2]
  private final float sI2 = 32*10e-6 ; //  moment of inertia of reaction wheel [kg. m^2]
  private final float sd11 = 4.83*10e-3 ; //  moment of inertia of entire system [kg. m^2]
  private final float sd12 = 32*10e-6;    // also moment of inertia of reaction wheel [kg. m^2]
  private final float sd21 = 32*10e-6;    // also moment of inertia of reaction wheel [kg. m^2]
  private final float sd22 = 32*10e-6;    //   moment of inertia of reaction wheel [kg. m^2]
  private final float sdetD = 155*10e-9 ; // determinant of the parameters of system moment of inertia, within EOM. [kg. m^2]
  private final float sm_bar = 38.7*10e-3;// sum of mass times distance to cg of each component, pendulum arm and reaction wheel. [kg]
  private final float sKm = 5.5*10e-3 ;   // Lumped gain representing motor torque constant and amplifier transconductance. [Nm/V]
  private final float SCALE_BASE_RADIUS = 50.0; // this will be our base plate for holding pendulum.
  private final float SCALE_BLADE_RADIUS = 50.0;  // Think in feet for helicopter similar to MH-65D. Should be 20
  private final float SCALE_CG_Z_DISPL = 17; // Should be 10.
  private final float SCALE_FUSE_DISPL = 25; // Should be 400/33=12.12121... 
  // Note below I've never done anything of this sort.

  public Pendulum(float s) {   // Object definition.
  
    // Build the MR controller actuation
    controller = new PVector();
    controller2 = new PVector();
    
  // Build the Pendulum. Initial placement.
    translation = new PVector(); 

    rotation = new PVector(); 
    
    baseJoint = new PVector[1];  
    bladeJoint = new PVector[4]; 
    q = new PVector[4]; 
    baseRadius = s*SCALE_BASE_RADIUS;  
    bladeRadius = s*SCALE_BLADE_RADIUS;
    cg_displ    = s*SCALE_CG_Z_DISPL ;
    fuse_displ  = s*SCALE_FUSE_DISPL ;
    fuse_mass_radius=2.5;
    
     
    euquat = new PVector(1,0,0); 
    eurwquat = new PVector(0,1,0); 
    eumrquat = new PVector(0,0,1);
    // Note: iNote: this is the user defined desired vector we wish to actuate to!
    b0 = new PVector(1,0,0);
    brw0 = new PVector(0,1,0);
    bmr0 = new PVector(0,0,1);
    
    // Below is for the body frame traj error and current velocity. For use in body frame control.
    b0p = new PVector(0,0,0);
    b0v = new PVector(0,0,0);
    // vector for finding error frame basis
    uu = new PVector(0,0,0);
    // vector for detecting desired frame rotation rate for F.F. compensator.
    uuu= new PVector(0,0,0);
    // erquat.
    erquat = new PVector(1,0,0);
    // past discreet time value.
    b0m1 = new PVector(1,0,0);
    // Note: the quaternion error between desired position and current  Euler determined  position, (In the error Reference frame rotating with b0!!!
    erquatErr = new PVector(1,0,0); // Initially set to zero.
    // To get body frame error wrt body frame.
    bodyFrameTrajError= new PVector(0,0,0);
    bodyFrameVelError = new PVector(0,0,0);
    globalPitch = new PVector(0,0,0);
    quatManeuv=  new PVector(0,0,0);
     // desired frame angular rate
    desOm = new PVector(0,0,0);
    // Past time desired frame angular rate, 
    desOmm1 = new PVector(0,0,0);
    // desired frame angular acceleration
    desOmDot = new PVector(0,0,0);

  // This is stable.
    for (int i=0; i<1; i++) 
    {  //  Base joint location calculation, B, in center of the base platform.
      float mx = 0; 
      float my = 0;
      baseJoint[i] = new PVector(mx, my, 0); 
    } // bon.
    
    for (int i=0; i<4; i++) { 
     float mx = bladeRadius*cos(radians(bladeAngles[i])); 
     float my = bladeRadius*sin(radians(bladeAngles[i]));
     float mz = 0;
      bladeJoint[i] = new PVector(mx, my, mz);  
      q[i] = new PVector(0, 0, 0); 
    }
    
    calcQ() ; 
  }    
 



  // Function below called in the main simulator file. Used to actuate the MR object using dynamics.
  public void applyTranslationAndRotation(PVector t, PVector r, PVector u, PVector v, PVector rr, PVector vv, PVector aa, PVector bb) {   // Here we set rotation vector and translation vector desired.
    rotation.set(r); 
    translation.set(t); 
    controller.set(u);
    controller2.set(v);
    b0.set(rr);
    b0p.set(aa);
    b0v.set(vv);
    quatManeuv.set(bb);
    
    float b0Norm= sqrt (b0.x*b0.x + b0.y*b0.y + b0.z*b0.z);
      if(b0Norm!=0){
         b0.x /= (float)b0Norm;
         b0.y /= (float)b0Norm;
         b0.z /= (float)b0Norm;
      }
      if(b0Norm==0){  // Here prevents getting erronous values for desired orientation
         b0.x = 0;
         b0.y = 0;
         b0.z = 1;
      }
     float euqNorm = sqrt(euquat.x*euquat.x + euquat.y*euquat.y + euquat.z*euquat.z) ; 
     euquat.x /= euqNorm;
     euquat.y /= euqNorm;
     euquat.z /= euqNorm; // */
     
      // Update old desOm.
     if(key2>1.0){
        desOmm1.y = desOm.y;
        desOmm1.z = desOm.z;
      }
   
     // ----Below is the frame rotation stuff-----//
     uu.x= b0.y;
     uu.y= -b0.x;
     uu.z= 0;
   
     uuu.x= euquat.y;
     uuu.y= -euquat.x;
     uuu.z= 0;
     
     float uuNorm= sqrt(uu.x*uu.x + uu.y*uu.y + uu.z*uu.z);
     float uuuNorm= sqrt(uuu.x*uuu.x + uuu.y*uuu.y + uuu.z*uuu.z);
     uu.x /=uuNorm;
     uu.y /=uuNorm;
     uu.z /=uuNorm;
     uuu.x /=uuuNorm;
     uuu.y /=uuuNorm;
     uuu.z /=uuuNorm;
     // 
     // Rotate b0 about uu to get the basis bmr0. Rotate 90 degrees.
     bmr0.x = uu.x*uu.x*b0.x + (uu.x*uu.y-uu.z)*b0.y + (uu.x*uu.z+uu.y)*b0.z;
     bmr0.y = (uu.x*uu.y+uu.z)*b0.x + uu.y*uu.y*b0.y + (uu.y*uu.z-uu.x)*b0.z;
     bmr0.z = (uu.x*uu.z-uu.y)*b0.x + (uu.y*uu.z+uu.x)*b0.y + uu.z*uu.z*b0.z;
     
     brw0.x = -1*(-b0.z*bmr0.y + b0.y*bmr0.z);
     brw0.y =  -1*(b0.z*bmr0.x - b0.x*bmr0.z);
     brw0.z = -1*(-b0.y*bmr0.x + b0.x*bmr0.y);
     
     // Rotate 90 degrees the euquat in direction of inertial "k" basis vector.
     float temp1x = uuu.x*uuu.x*euquat.x + (uuu.x*uuu.y-uuu.z)*euquat.y + (uuu.x*uuu.z+uuu.y)*euquat.z;
     float temp1y = (uuu.x*uuu.y+uuu.z)*euquat.x + uuu.y*uuu.y*euquat.y + (uuu.y*uuu.z-uuu.x)*euquat.z;
     float temp1z = (uuu.x*uuu.z-uuu.y)*euquat.x + (uuu.y*uuu.z+uuu.x)*euquat.y + uuu.z*uuu.z*euquat.z;
     
     // Rotate the before about equat, wrt the inertial roll angle.
     eumrquat.x = (euquat.x*euquat.x*(1-cos(-rotation.x))+cos(-rotation.x))*temp1x + (euquat.x*euquat.y*(1-cos(-rotation.x))-euquat.z*sin(-rotation.x))*temp1y + (euquat.x*euquat.z*(1-cos(-rotation.x))+euquat.y*sin(-rotation.x))*temp1z ;   
     eumrquat.y = (euquat.x*euquat.y*(1-cos(-rotation.x))+euquat.z*sin(-rotation.x))*temp1x + (euquat.y*euquat.y*(1-cos(-rotation.x))+cos(-rotation.x))*temp1y + (euquat.y*euquat.z*(1-cos(-rotation.x))-euquat.x*sin(-rotation.x))*temp1z ;   
     eumrquat.z = (euquat.x*euquat.z*(1-cos(-rotation.x))-euquat.y*sin(-rotation.x))*temp1x + (euquat.y*euquat.z*(1-cos(-rotation.x))+euquat.x*sin(-rotation.x))*temp1y + (euquat.z*euquat.z*(1-cos(-rotation.x))+cos(-rotation.x))*temp1z ;
    //*/
  
     float eumrNorm = sqrt(eumrquat.x*eumrquat.x+eumrquat.y*eumrquat.y+eumrquat.z*eumrquat.z);
     float eurwNorm = sqrt(eurwquat.x*eurwquat.x+eurwquat.y*eurwquat.y+eurwquat.z*eurwquat.z);
     
     eumrquat.x /= eumrNorm;
     eumrquat.y /= eumrNorm;
     eumrquat.z /= eumrNorm;
   
     eurwquat.x /= eurwNorm;
     eurwquat.y /= eurwNorm;
     eurwquat.z /= eurwNorm;
     
     // Take cross product to calculate
     eurwquat.x = (-euquat.y*eumrquat.z + euquat.z*eumrquat.y);
     eurwquat.y = ( euquat.x*eumrquat.z - euquat.z*eumrquat.x);
     eurwquat.z = (-euquat.x*eumrquat.y + euquat.y*eumrquat.x);
     
     // Renormalize.
     eurwNorm = sqrt(eurwquat.x*eurwquat.x+eurwquat.y*eurwquat.y+eurwquat.z*eurwquat.z);
     eurwquat.x /= eurwNorm;
     eurwquat.y /= eurwNorm;
     eurwquat.z /= eurwNorm;
     
   
   // ---- Below is end of the frame rotation stuff-----//
   
   
   calcQ(); // set the Ikine simultaneously to show pretty picture.
   
  }
  
  
  
  private void calcQ() {        
    // Set the 'q' vector rotation.
    for (int i=0; i<4; i++) { 

        // NOTE: below contains dynamics for actuating MR rotate, cyclic and collective actions as function of swashplate.
        // NOTE: We must initiallize the beta in the main simulation.   via the variables A1, B1, theta0.
                                                   
        q[i].x =  cos(controller2.z + bladeAngles[i]*PI/180.0 ) * bladeRadius; //
                                                         // bon
        q[i].y = -sin(controller2.z + bladeAngles[i]*PI/180.0 ) * bladeRadius; // 
  
        q[i].z = sin( controller.z/1 - controller.x*PI/180*sin(controller2.z + bladeAngles[i]*PI/180.0) + controller.y*PI/180*cos(controller2.z + bladeAngles[i]*PI/180.0) ) * bladeRadius;   
          
    }
    
       quatRotOpsVecProdF(); // Calculates the desired rotation frames.
  }  
 
  
  
  private void quatRotOpsVecProdF() {  
   // Note: Below is so fluggedup!
     euquat.x = cos(rotation.z)*cos(rotation.y);
     euquat.y = -sin(rotation.z)*cos(rotation.y);
     euquat.z = sin(rotation.y);  // */

     // Express the current A/C body frame unit vectors in the rotating desired frame.
     erquat.x= euquat.x*b0.x  + euquat.y*b0.y  + euquat.z*b0.z;
     erquat.y= euquat.x*brw0.x+euquat.y*brw0.y+euquat.z*brw0.z;
     erquat.z= euquat.x*bmr0.x+euquat.y*bmr0.y+euquat.z*bmr0.z;
    
    // If this first time itteration equate past value of euquat as the current value
      if(key1<1.0){ 
          b0m1.x= b0.x;
          b0m1.y= b0.y;
          b0m1.z= b0.z;
      }
      key1 = 2.0; 
      
    float b0bx= (euquat.x*b0.x + -eurwquat.x*b0.y + eumrquat.x*b0.z);
    float b0by= (euquat.y*b0.x + -eurwquat.y*b0.y + eumrquat.y*b0.z);
    float b0bz= (euquat.z*b0.x + -eurwquat.z*b0.y + eumrquat.z*b0.z);
    
    float b0m1bx= (euquat.x*b0m1.x + -eurwquat.x*b0m1.y + eumrquat.x*b0m1.z); 
    float b0m1by= (euquat.y*b0m1.x + -eurwquat.y*b0m1.y + eumrquat.y*b0m1.z);
    float b0m1bz= (euquat.z*b0m1.x + -eurwquat.z*b0m1.y + eumrquat.z*b0m1.z);     
    
    float thetat2 = atan2(b0bz,sqrt(b0bx*b0bx + b0by*b0by)) ;
    float psit2 =   -atan2(b0by,b0bx)       ;
    float thetat3 = atan2(b0m1bz,sqrt(b0m1bx*b0m1bx + b0m1by*b0m1by)) ;
    float psit3 =   -atan2(b0m1by,b0m1bx)   ;
    
    // Extract desired frame angular rotation.
    desOm.y = (thetat2-thetat3)/.05;
    desOm.z = (psit2-psit3)/.05;
    
    // Neccesary initialization.
    if(key2<1.0){
      desOmm1.y = desOm.y;
      desOmm1.z = desOm.z;
    }
    key2=2.0; // end the initialization.
    
    // Extract desired frame angular rotation.
    desOmDot.y = (desOm.y-desOmm1.y)/.05;
    desOmDot.z = (desOm.z-desOmm1.z)/.05;
  }
  
  public PVector getErQuatError(){
  
    float pk0 = 0;  // Make zero for pure quaternion.
    float pk1= 1;  
    float pk2= 0;
    float pk3= 0;
    
    // Compute measured quaternion conjugate.
    float q0 = 0; 
    float q1= -erquat.x;  
    float q2= -erquat.y;
    float q3= -erquat.z;


    // qxyz calculation. Yaw -> Pitch -> Roll 
    float r0 = pk0*q0+-pk1*q1+-pk2*q2+-pk3*q3;  
    float r1 = pk1*q0+pk0*q1+-pk3*q2+pk2*q3;
    float r2 = pk2*q0+pk3*q1+pk0*q2+-pk1*q3;
    float r3 = pk3*q0+-pk2*q1+pk1*q2+pk0*q3;
    
    // Update quaternion Error.
    erquatErr.x= r1;  
    erquatErr.y= r2; 
    erquatErr.z= r3;
  
    return erquatErr; 
  } // */
  
  
  
  public float getTargYaw(){
    float b0planx,b0plany,b0planz;
    float b0planNorm;
    float euquatplanx, euquatplany, euquatplanz;
    float euquatplanNorm;
    
    b0planx= b0.x; b0plany= b0.y; b0planz= 0;
    b0planNorm= sqrt(b0planx*b0planx+b0plany*b0plany+b0planz*b0planz);
    if(b0planNorm!=0){ // prevent division by 0.
        b0planx /=b0planNorm; b0plany /=b0planNorm; b0planz /=b0planNorm;
    }
    
    euquatplanx= erquat.x; euquatplany= erquat.y; euquatplanz= 0;
    euquatplanNorm= sqrt(euquatplanx*euquatplanx+euquatplany*euquatplany+euquatplanz*euquatplanz);
    if(euquatplanNorm!=0){ // prevent division by 0.
        euquatplanx /=euquatplanNorm; euquatplany /=euquatplanNorm; euquatplanz /=euquatplanNorm;
    }
    
    if(abs(b0.x)+abs(b0.y)>0){ 
        yawErr=-atan2(0,1)+atan2(euquatplany,euquatplanx);
    }
    
    if(sqrt(b0.x*b0.x+b0.y*b0.y)< .1){ // Turn off yaw control if the desired target quaternion near sinularity direction.
      yawErr = 0;
    } // */
    return yawErr;
  } // */
  
  
  public PVector getBodyFrameTrajError(){
    float delX, delY, delAlt;


    delX= b0p.x*cos(rotation.z)*cos(rotation.y) + b0p.y*(cos(rotation.z)*sin(rotation.y)*sin(rotation.x)-sin(rotation.z)*cos(rotation.x)) + b0p.z*(cos(rotation.z)*sin(rotation.y)*cos(rotation.x)+sin(rotation.z)*sin(rotation.x));
    delY= b0p.x*sin(rotation.z)*cos(rotation.y) + b0p.y*(sin(rotation.z)*sin(rotation.y)*sin(rotation.x)+cos(rotation.z*cos(rotation.x)) ) + b0p.z*(sin(rotation.z)*sin(rotation.y)*cos(rotation.x)-cos(rotation.z)*sin(rotation.x));
    delAlt= -b0p.x*(sin(rotation.y)) + b0p.y*(cos(rotation.y)*sin(rotation.x))  + b0p.z*(cos(rotation.y)*cos(rotation.x)); // */
      
    bodyFrameTrajError.x= delX;
    bodyFrameTrajError.y= delY;
    bodyFrameTrajError.z= delAlt;
    return bodyFrameTrajError;
  }

  // Here I receive inertial frame velocity and compute body frame velocity.
  public PVector getBodyFrameVelError(){
      float delX, delY, delAlt;
      b0v.x *= 1; b0v.y *= 1; b0v.z *= 1;
      
     
     delX= b0v.x*cos(rotation.z)*cos(rotation.y) + b0v.y*(cos(rotation.z)*sin(rotation.y)*sin(rotation.x)-sin(rotation.z)*cos(rotation.x)) + b0v.z*(cos(rotation.z)*sin(rotation.y)*cos(rotation.x)+sin(rotation.z)*sin(rotation.x));
      delY= b0v.x*sin(rotation.z)*cos(rotation.y) + b0v.y*(sin(rotation.z)*sin(rotation.y)*sin(rotation.x)+cos(rotation.z*cos(rotation.x)) ) + b0v.z*(sin(rotation.z)*sin(rotation.y)*cos(rotation.x)-cos(rotation.z)*sin(rotation.x));
      delAlt= -b0v.x*(sin(rotation.y)) + b0v.y*(cos(rotation.y)*sin(rotation.x))  + b0v.z*(cos(rotation.y)*cos(rotation.x)); // */
      
      bodyFrameVelError.x= delX;
      bodyFrameVelError.y= delY;
      bodyFrameVelError.z= delAlt;
      return bodyFrameVelError;
    }
    
     public PVector getb0rw(){
   
      return brw0;
    }
    
    public PVector getDesOmegaDot(){
    return desOmDot;
  } // */
    

 //<>//
  public void draw() {   
   pushMatrix();
     //rotateX(PI/2);   // Yes.
     translate(0,0,-7.5);
     //noStroke(); 
     stroke(200);
     strokeWeight(0.5);
    fill(50);
     ellipse(0, 0, 15*baseRadius, 15*baseRadius); 
     // noStroke(); 
     stroke(200);
     strokeWeight(0.5);
     translate(0,0,1);
    fill(25);
    ellipse(0, 0, 10*baseRadius,10*baseRadius); 
     // noStroke(); 
     stroke(200);
     strokeWeight(0.5);
     translate(0,0,1);
    fill(10);
    ellipse(0, 0, 5*baseRadius,5*baseRadius); 
   popMatrix();



   pushMatrix();
  
       // Main translation code for inertial frame movement.
       translate(translation.x,translation.y,translation.z); 
          pushMatrix();
            strokeWeight(2);
            //stroke(78);
            stroke(255);
            translate(0,0,0);
            line(0, 0, 0, 220*eurwquat.x, 220*eurwquat.y, 220*eurwquat.z);
          popMatrix(); // */
          
          // Draw the vector normal to our nose quaternion. 
          pushMatrix();
            strokeWeight(2);
            //stroke(78);
            stroke(10, 200, 200);
            translate(0,0,0);
            line(0, 0, 0, 220*eumrquat.x, 220*eumrquat.y, 220*eumrquat.z);
          popMatrix(); // */
         
         // Draw the vector for the Euler angle actuated of the A/C
          pushMatrix();
            strokeWeight(2);
            //stroke(78);
            stroke(182, 86, 93);
            translate(0,0,0);
            line(0, 0, 0, 400*euquat.x, 400*euquat.y, 400*euquat.z);
          popMatrix(); // */
         
         float b0Norm= sqrt (b0.x*b0.x + b0.y*b0.y + b0.z*b0.z);
         b0.x /= (float)b0Norm;
         b0.y /= (float)b0Norm;
         b0.z /= (float)b0Norm; 
         stroke(0);
         
          // Draw the vector normal to our nose quaternion.  
          pushMatrix();
            strokeWeight(4);
            translate(0,0,0);
            stroke(0);
            line(0, 0, 0, 100*b0.x, 100*b0.y, 100*b0.z);
          popMatrix(); 
          pushMatrix();
            strokeWeight(3);
            translate(0,0,0);
            stroke(0);
            line(0, 0, 0, 25*bmr0.x, 25*bmr0.y, 25*bmr0.z);
          popMatrix(); 
          pushMatrix();
            strokeWeight(3);
            translate(0,0,0);
            stroke(0);
            line(0, 0, 0, 25*brw0.x, 25*brw0.y, 25*brw0.z);
          popMatrix(); //*/
    // Here is the end of the imaging of the frame rotation stuff.
       
       translate(0,0,fuse_displ-cg_displ);
       
       // Note: the  lines below. Ensure that rotation is about the C.G.  location in the body axis frame. 
       rotateZ(-rotation.z);
       rotateY(-rotation.y);   
       rotateX(-rotation.x); 
       translate(0,0,cg_displ);  // Move the Heli up so fuselage mass sits.
       
       rotateZ(PI/2.0);  // to ensure the Helicopter is initially in inertial frame and makes sense.  
                         // +A1 == +x-dir    +B1 == +y-dir
       
           pushMatrix();
              noStroke();
               fill(50);
              ellipse(0, 0, 5,5);
              stroke(255);
              strokeWeight(4);
             line(0,0,0, 0, 0, -fuse_displ);
             
             pushMatrix();
               translate(0,0,-fuse_displ/3);
               stroke(0);
               sphere(1);
               translate(0,0,-fuse_displ/3);
               sphere(1);
               translate(0,0,2*fuse_displ/3);
             popMatrix(); // */
             
             pushMatrix();
               strokeWeight(4);
               stroke(0);
               line(0,0,-fuse_displ/3,0,fuse_displ/3*sqrt(3)/2,-fuse_displ/2);
               line(0,0,-2*fuse_displ/3,0,fuse_displ/3*sqrt(3)/2,-fuse_displ/2);
               line(0,fuse_displ/3*sqrt(3)/2,-fuse_displ/2, 0,2.5*fuse_displ,-fuse_displ/2);
               translate( 0,2.5*fuse_displ,-fuse_displ/2);
               
               stroke(250);
               noFill();
               rotateY(PI/2);
               ellipse(0,0,15,15);
             popMatrix();
             
             
             translate(0,0,-cg_displ); // Move drawing origin to cg location.         
             fill(10);
             noStroke();
             sphere(1);
             translate(0,0,-(fuse_displ-cg_displ)); 
             fill(100);
             sphere(fuse_mass_radius*2.0);
           popMatrix();
           
           
         // DRAW THE TAIL ROTOR BLADES.
          for (int i=0; i<4; i++) {
               pushMatrix();
               translate(0,2.5*fuse_displ,-fuse_displ/2.0);
               rotateY(PI/2);
                   rotateZ(-controller2.z-tailAngles[i]*PI/180.0); 
                   rotateX(-controller2.y);
                   
                   // Blade drawing.
                   fill(100);
                   noStroke();
                   ellipse(7.50/2.0,0,7.50,2);
                   
               popMatrix();
            }
         // 
           
            // DRAW THE MAIN ROTOR BLADES 
          //   Actuated by the cyclic and the collective controller.
            for (int i=0; i<4; i++) {
              pushMatrix();
               
                  pushMatrix();
                   rotateZ(-controller2.z-bladeAngles[i]*PI/180.0);
                   rotateY( -(controller.z/1 + controller.y*PI/180*sin(controller2.z + bladeAngles[i]*PI/180.0) - controller.x*PI/180*cos(controller2.z + bladeAngles[i]*PI/180.0)) );  
                   
                   // Implement BLADE FEATHERING. 
                   rotateX((-controller.z/1 + controller.y*PI/180*cos(controller2.z + bladeAngles[i]*PI/180.0) + controller.x*PI/180*sin(controller2.z + bladeAngles[i]*PI/180.0) ) ); 
                           
                   // Blade drawing.
                   fill(100);
                   noStroke();
                   ellipse(bladeRadius/2.0,0,bladeRadius,5);
                     
                   popMatrix();
                
              popMatrix();
            }
            
            // DRAW THE ROTOR DISC PLANE
                     rotateY(  controller.x*PI/180 );
                     rotateX(-controller.y*PI/180);      
                     rotateZ(-controller2.z);
                      
                     fill(255);
                     noStroke();
                     translate(0, 0, -5);
                     ellipse(0,0, 20, 20);
   popMatrix();   
   
   // Draw the inertial frame basis.
     pushMatrix();
       stroke(10, 200, 200);
       line(0, 0, 0, 0,0,100);
       stroke(182, 86, 93);
       line(0, 0, 0, 100,0,0);
       stroke(255);
       line(0, 0, 0, 0,100,0);
         
     popMatrix();
   
   
    stroke(255);

   
  }
}



        
        
        
