// Real-time MPC controller on Galileo 
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=// 
// Initializing of MPC  
var Ap = [[0.9843 , 0.0000], [0.2498 , 0.9932]];
var Bp = [[0.2048], [0.0259]];
var Cp = [[0.0000 , 0.2500]];
var n = 2 ;
var k = 1 ;
var m = 1 ;
var Np = 1 ;
var Nc = 1 ;
var u_min = 1 ;
var u_max = 1 ;
var delta_U_min = 1 ;
var delta_U_max = 1 ;
var y_min = 1 ;
var y_max = 1 ;
var ref = 1 ;
var pwmPin1 = 2 ;
var SensorPin1 = 3 ;

var mraa = require('mraa'); //require mraa
console.log('MRAA Version: ' + mraa.getVersion()); //write the mraa version to the Intel XDK console

// kalman variables
    var Pc = 0.0;
    var G  = 0.0;
    var P  = 1.0;
    var Xp = 0.0;
    var Zp = 0.0;
    var Xe = 0.0;

//Initialize PWM
    var pwm = new mraa.Pwm(pwmPin1);
//Enable PWM
    pwm.enable(true);
    var TL=18000;
    pwm.period_us(TL);//set the period in microseconds.	
    pwm.write(0);

//analog sensor 
    var Sensor = new mraa.Aio(SensorPin1); //setup access analog input Analog pin #0 (A0)

//mpc controller configuration
    var control_A=0; //value of PWM
    var SRead1 = 0;
    var sensorRead1 ;
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=//
// MPC controller
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=//
// MPC gain constants initializations
    var A_e = [];
    var B_e = [];
    var C_e = [];
    var Phi_Phi = [];
    var Phi_F = [];
    var Phi_R = [];
//--------------------------------------------------------------------//
    // Calculate the MPC Gains
    mpcgain(Ap,Bp,Cp,n,m,k,Nc,Np);

    var xm = [];
    for (var i = 0 ; i < n ; i++)
        xm.push([]);
    for (var i = 0 ; i < n; i++)
        for (var j = 0; j < k ; j++)
            xm[i][j] = 0;

    var xm_old = [];
    for (var i = 0 ; i < n ; i++)
        xm_old.push([]);
    for (var i = 0 ; i < n; i++)
        for (var j = 0; j < k ; j++)
            xm_old[i][j] = 0;

    var Xf = [];
    for (var i = 0 ; i < n+m ; i++)
        Xf.push([]);
    for (var i = 0 ; i < n+m; i++)
        for (var j = 0; j < k ; j++)
            Xf[i][j] = 0;

    var delta_U = [];
    for (var i = 0 ; i < Nc ; i++)
        delta_U.push([]);

    var U = 0;
    var y = 0;

// Generating Matrix R_Bar
    var R_eye = matrix_eye(Nc);
     
    var R_Bar = [];
    for (var i = 0 ; i < Nc ; i++)
        R_Bar.push([]);
    for (var i = 0 ; i < Nc ; i++)
        for(var j = 0 ; j < Nc ; j++)
            R_Bar[i][j] = 2*R_eye[i][j];

    var Phi_R_Bar = [];
    Phi_R_Bar = matrix_add(Nc,Nc,Phi_Phi,R_Bar);
    Phi_R_Bar = matrix_inv(Nc,Phi_R_Bar); // **
    
    var r_Phi_R = [];
    for (var i = 0 ; i < Nc ; i++)
        r_Phi_R.push([]);
    for (var i = 0 ; i < Nc ; i++)
        for(var j = 0 ; j < k ; j++)
            r_Phi_R[i][j] = ref*Phi_R[i][j];

mpc_control();

function mpc_control (){
    setInterval(function(){
        var temp = [];
        var temp1 = [];
        
        temp = matrix_mul(Nc,m+n,1,Phi_F,Xf);
        temp1 = matrix_sub(Nc,m,r_Phi_R,temp);// **
        
        delta_U = matrix_mul(Nc,Nc,m,Phi_R_Bar,temp1);
        
        /* Check control limits */
        if( delta_U[0][0] > delta_U_max ) 
            delta_U[0][0] = delta_U_max;
        
        if( delta_U[0][0] < delta_U_min ) 
            delta_U[0][0] = delta_U_min;
        
        U = U + delta_U[0][0];
        
        /* Check control limits */
        if( U > u_max ) U = u_max;
        if( U < u_min ) U = u_min;
        
        xm_old = xm;
        // Calculate => xm = Ap*xm + Bp*U
        temp = matrix_mul(n,n,k,Ap,xm);
        for (var i = 0 ; i < n ; i++)
            temp1.push([]);
        for (var i = 0 ; i < n ; i++)
            for(var j = 0 ; j < k ; j++)
                temp1[i][j] = U*Bp[i][j];
        xm = matrix_add(n,k,temp,temp1);
       
        y = readSensor();
        /* Checkoutput limits */
        if( y > y_max ) y = y_max;
        if( y < y_min ) y = y_min;
    
        temp = matrix_sub(n,k,xm,xm_old);
        
        for (var i = 0 ; i < n ; i++)
            Xf[i][0] = temp[i][0];
        
        Xf[n+m-1][0] = y;
        
        console.log(delta_U[0][0]);
        console.log(U);
        applyVolt(U); 
    },1000);
}
//==============================================================================//
/* For Calculating MPC Gains*/
function mpcgain(Ap,Bp,Cp,n,m,k,Nc,Np){
    // Temporary matrices
    var temp = [];
    var temp1 = [];
    
    // Generating MAtrix A_e
    A_e = matrix_eye(n+m);
    for (var i = 0 ; i < n; i++)
        for (var j = 0; j < n ; j++)
            A_e[i][j] = Ap[i][j];
    
    temp = matrix_mul(m,n,n,Cp,Ap);
    for (var i = n ; i < n+m; i++)
        for (var j = 0; j < n ; j++)
            A_e[i][j] = temp[i-n][j];
    
    // Generating matrix B_e
    for (var z = 0 ; z < n+m; z++)
        B_e.push([]);
    
   for (var i = 0 ; i < n+m; i++)
        for (var j = 0; j < k ; j++)
            B_e[i][j] = 0;
    for (var i = 0 ; i < n; i++)
        for (var j = 0; j < k ; j++)
            B_e[i][j] = Bp[i][j];
    
    temp = matrix_mul(m,n,k,Cp,Bp);
    for (var i = n ; i < n+m; i++)
        for (var j = 0; j < k ; j++)
            B_e[i][j] = temp[i-n][j];
    
    // Generating matrix C_e
    for (var z = 0 ; z < m ; z++)
        C_e.push([]);
    
   for (var i = 0 ; i < m; i++)
        for (var j = 0; j < m+n ; j++)
            C_e[i][j] = 0;

    temp = matrix_eye(m);
    for (var i = 0 ; i < m; i++)
        for (var j = n; j < n+m ; j++)
            C_e[i][j] = temp[i][j-n];
    
    // Generating matrix h
    var h = [];
    for (var z = 0 ; z < Np ; z++)
        h.push([]);
    
    for (var i = 0 ; i < 1; i++)
        for (var j = 0; j < n+m ; j++)
            h[i][j] = C_e[i][j];
    
    for (var i = 0 ; i < Np-1 ; i++){
        for (var z = 0 ; z < m+n ; z++)
            temp1.push([]);
        for (var q = 0 ; q < 1 ; q++)
            for (var j = 0 ; j < n+m ; j++)
                temp1[q][j] = h[i][j];
        
        temp = matrix_mul(m,n+m,n+m,temp1,A_e);
        
        for (var q = 0 ; q < 1 ; q++)        
            for (var j = 0 ; j < n+m ; j++)
                h[i+1][j] = temp[q][j];
    }
    
    // Generating matrix F
    var F = [];
    for (var z = 0 ; z < Np ; z++)
        F.push([]);
    
    temp = matrix_mul(m,n+m,n+m,C_e,A_e)
    for (var i = 0 ; i < 1; i++)
        for (var j = 0; j < n+m ; j++)
            F[i][j] = temp[i][j];
    
    for (var i = 0 ; i < Np-1 ; i++){
        for (var z = 0 ; z < m+n ; z++)
            temp1.push([]);
        for (var q = 0 ; q < 1 ; q++)
            for (var j = 0 ; j < n+m ; j++)
                temp1[q][j] = F[i][j];
        
        temp = matrix_mul(m,n+m,n+m,temp1,A_e);
        
        for (var q = 0 ; q < 1 ; q++)        
            for (var j = 0 ; j < n+m ; j++)
                F[i+1][j] = temp[q][j];
    }
    
    // Generating matrix v
    var v = [];
    
    v = matrix_mul(Np,n+m,k,h,B_e);
    
    // Generating Matrix Phi
    var Phi = [];
    for (var z = 0 ; z < Np ; z++)
        Phi.push([]);
    
    for (var i = 0 ; i < Np; i++)
        for (var j = 0; j < Nc ; j++)
            Phi[i][j] = 0;
    
    for (var countNc = 0 ; countNc < Nc ; countNc++)
        for (var j = 0 ; j < Np-countNc ; j++)
            Phi[j+countNc][countNc] = v[j][0];
    
    // Generating Matrix RsBar
    var RsBar = [];
    for (var z = 0 ; z < Np ; z++)
        RsBar.push([]);
    
    for (var i = 0 ; i < Np; i++)
        RsBar[i][0] = 1;
    
    // Calculate => Phi_Phi = Phi'*Phi
    var PhiTrans = [];

    PhiTrans = matrix_trans(Np,Nc,Phi);
    Phi_Phi = matrix_mul(Nc,Np,Nc,PhiTrans,Phi);
    
    // Calculate => Phi_F = Phi'*F
    Phi_F = matrix_mul(Nc,Np,n+m,PhiTrans,F);
    
    // Calculate => Phi_R = Phi'*RsBar
    Phi_R = matrix_mul(Nc,Np,1,PhiTrans,RsBar);
}
//====================================================================================
/* For Reading the Value form the Sensor */
function readSensor(){
    sensorRead1 = Sensor.read(); //read the value of the analog pin
    //console.log('Analog Sensor1= '+sensorRead1); //write the value of the analog pin to the console
    
    var a=  -0.0005344 ;
    var b=  0.7003 ;
    var c=  -190.5;
    SRead1 = (a*sensorRead1*sensorRead1) + (b*sensorRead1) + c;
		
    // kalman process
    Pc = P + varProcess;
    G = Pc/(Pc+varVolt); // kalman gain
    P = (1-G)*Pc;
    Xp = Xe;
    Zp = Xp;
    Xe = G*(SRead1-Zp)+Xp;   // the kalman estimate of the sensor voltage
    //console.log(sensorRead1);
    console.log(SRead1); //write the value of the sensor to the console
    console.log(Xe); //write the value of the KF to the console
    
   return Xe;
}
//====================================================================================
/* For Applying the voltage to the Pump */
function applyVolt(VL){
    control_A = VL;
    pwm.write(control_A); //Write duty cycle value. 	
}
//====================================================================================
/* For Calculating the Multiplication of Two Matrices */
function matrix_mul(n,m,p,mat1,mat2){
    var sum = 0;
    var mul = [];
    for (var i = 0 ; i < n ; i++)
        mul.push([]);
    
    for (var c = 0; c < n; c++) {
      for (var d = 0; d < p; d++) {
        for (var k = 0; k < m; k++) {
          sum = sum + mat1[c][k]*mat2[k][d];
        }
 
        mul[c][d] = sum;
        sum = 0;
      }
    }
    return mul;
}
//====================================================================================
/* For Calculating the Addition of Two Matrices */
function matrix_add(n,m,mat1,mat2){
    var add = [];
    for (var i = 0 ; i < n ; i++)
        add.push([]);
    
    for (var c = 0; c < n; c++) {
        for (var k = 0; k < m; k++) {
          add[c][k] = mat1[c][k]+mat2[c][k];
        }
    }
    return add;
}
//====================================================================================
/* For Calculating the Subtraction of Two Matrices */
function matrix_sub(n,m,mat1,mat2){
    var sub = [];
    for (var i = 0 ; i < n ; i++)
        sub.push([]);
    
    for (var c = 0; c < n; c++) {
        for (var k = 0; k < m; k++) {
          sub[c][k] = mat1[c][k]-mat2[c][k];
        }
    }
    return sub;
}
//====================================================================================
/* For Calculating the Transpose of the Matrix */
function matrix_trans (n,m,mat){
    var trans = [];
    for (var i = 0 ; i < m ; i++)
        trans.push([]);
    
    for (var c = 0; c < n; c++)
      for( var d = 0 ; d < m ; d++ )
         trans[d][c] = mat[c][d];
      
    return trans;
}
//====================================================================================
/* For Calculating Determinant of the Matrix */
function matrix_det(k,mat){
  var s = 1,det = 0;
  var matt = [];
    for (var i = 0 ; i < k ; i++)
        matt.push([]);
    
  var i, j, m, n, c;
  if (k === 1)
    {
     return (mat[0][0]);
    }
  else
    {
     det = 0;
     for (c = 0; c < k; c++)
       {
        m = 0;
        n = 0;
        for (i = 0;i < k; i++){
            for (j = 0 ;j < k; j++){
                matt[i][j] = 0;
                if (i !== 0 && j !== c){
                   matt[m][n] = mat[i][j];
                   if (n < (k - 2))
                    n++;
                   else{
                     n = 0;
                     m++;
                     }
                   }
               }
             }

          det = det + s * (mat[0][c] * matrix_det(k - 1,matt));
          s = -1 * s;
          }
    }
    return det;
}
//====================================================================================
/* For Calculating The Inverse of the Matrix */
function matrix_inv(f,mat){

 var b = [];
    for (var i = 0 ; i < f ; i++)
        b.push([]);
 var fac = [];
    for (var i = 0 ; i < f ; i++)
        fac.push([]);
 var temp = [];
  for (var i = 0 ; i < f ; i++)
      temp.push([]);
 var inv = [];
  for (var i = 0 ; i < f ; i++)
      inv.push([]);

 var p, q, m, n, i, j,d;

 for (q = 0;q < f; q++){
   for (p = 0;p < f; p++){

     m = 0;
     n = 0;
     for (i = 0;i < f; i++){
       for (j = 0;j < f; j++){
           if (i !== q && j !== p){
            b[m][n] = mat[i][j];
            if (n < (f - 2))
             n++;
            else{
               n = 0;
               m++;
               }
            }
        }
      }
      fac[q][p] = Math.pow(-1, q + p) * matrix_det(f - 1,b);
    }
  }
  temp=matrix_trans(f,f,fac);

  d = matrix_det(f, mat);
  
  for (i = 0;i < f; i++){
     for (j = 0;j < f; j++){
        inv[i][j] = temp[i][j] / d;
        }
    }
    return inv;
}
//====================================================================================
/* For Calculating Identity Matrix */
function matrix_eye(n){

    var i;
    var j;
    var eye = [];
    for (var i = 0 ; i < n ; i++)
        eye.push([]);

    for (i=0; i < n; i++){
      for (j=0; j < n; j++){
           if (i === j){
                eye[i][j]=1;
           }         
           else {
              eye[i][j]=0;
           }
       }
    }
    return eye;
}
