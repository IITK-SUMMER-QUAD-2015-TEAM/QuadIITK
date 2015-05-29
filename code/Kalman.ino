float magnetNormal[3],accelNormal[3];
float Kalman_roll=0, Kalman_pitch=0, Kalman_yaw=0;
float a=0.5, b=0.5, c=0.5, d=0.5;
float n_k[4][1], q[4][1], q_predicted[4][1], q_Update[4][1]={{0.5},{0.5},{0.5},{0.5}}, p_predicted[4][4], p_Update[4][4], q_Osserv[4][1]={{0.5},{0.5},{0.5},{0.5}};
float dt;


extern int16_t magnetRaw[3];
extern float accelRate[3];

int ab=0;


void kalman(){
    measureIMUSensors();
    getMagnet();
    normalizeMagAcc(magnetRaw[XAXIS],magnetRaw[YAXIS],magnetRaw[ZAXIS],accelRate[XAXIS],accelRate[YAXIS],accelRate[ZAXIS]);//normalize magnetometer and Acc values
    GaussNewtonMethod();        //computing qOsserv
    kalman_filtering();
    GetAnglesFromQuaternion();
}

void normalizeMagAcc(float Mx,float My,float Mz,float Ax,float Ay,float Az)
{
  double magnM= sqrt(Mx*Mx+My*My+Mz*Mz);
  double magnA= sqrt(Ax*Ax+Ay*Ay+Az*Az);
  
  magnetNormal[XAXIS]=Mx/magnM;
  magnetNormal[YAXIS]=My/magnM;
  magnetNormal[ZAXIS]=Mz/magnM;
  
  accelNormal[XAXIS]=Ax/magnA;
  accelNormal[YAXIS]=Ay/magnA;
  accelNormal[ZAXIS]=Az/magnA;
}

void GaussNewtonMethod(){
    float hTemp[4][1], h[4][1],q[4][1], q_conj[4][1], bMagn[4][1], norm_Mb, m[4][1], J[6][4], R[3][3],M[6][6];
    q_conj[0][0] =  q_Osserv[0][0];
    q_conj[1][0] = -q_Osserv[1][0];        
    q_conj[2][0] = -q_Osserv[2][0];        
    q_conj[3][0] = -q_Osserv[3][0];
    
    a = q_Osserv[1][0];
    b = q_Osserv[2][0];
    c = q_Osserv[3][0];
    d = q_Osserv[0][0];
    
    m[0][0] = 0;
    m[1][0] = magnetNormal[XAXIS];
    m[2][0] = magnetNormal[YAXIS];    
    m[3][0] = magnetNormal[ZAXIS];   


    QuaternionProduct(q_Osserv,m,hTemp);
    QuaternionProduct(hTemp, q_conj, h);

    bMagn[0][0] = sqrt( h[1][0]*h[1][0] + h[2][0]*h[2][0]);
    bMagn[1][0] = 0;
    bMagn[2][0] = h[3][0];
   
    norm_Mb = sqrt(bMagn[0][0]*bMagn[0][0] + bMagn[1][0]*bMagn[1][0] + bMagn[2][0]*bMagn[2][0]);  
    
    bMagn[0][0] = bMagn[0][0]/norm_Mb;
    bMagn[1][0] = bMagn[1][0]/norm_Mb;
    bMagn[2][0] = bMagn[2][0]/norm_Mb;

    compute_Jacobian(J, a, b, c, d,accelRate[XAXIS],accelRate[YAXIS],accelRate[ZAXIS],magnetRaw[XAXIS],magnetRaw[YAXIS],magnetRaw[ZAXIS]);  
    ComputeR_Matrix(R, a, b, c, d);
    ComputeM_Matrix(M, R);
    
    float y_e[6][1], y_b[6][1];
    
    y_e[0][0] = 0;
    y_e[1][0] = 0;
    y_e[2][0] = -1;
    y_e[3][0] = bMagn[0][0];
    y_e[4][0] = bMagn[1][0];
    y_e[5][0] = -bMagn[2][0];   
   
    y_b[0][0] = accelNormal[XAXIS]; 
    y_b[1][0] = accelNormal[YAXIS];
    y_b[2][0] = accelNormal[ZAXIS];
    y_b[3][0] = magnetNormal[XAXIS];
    y_b[4][0] = magnetNormal[YAXIS];
    y_b[5][0] = magnetNormal[ZAXIS];
   
   n_k[0][0] = a;
   n_k[1][0] = b;
   n_k[2][0] = c;
   n_k[3][0] = d;   
   
   Gauss_Newton_step(n_k, y_e, y_b, J,M);
   
   float norm;
   norm = sqrt(n_k[0][0]*n_k[0][0] + n_k[1][0]*n_k[1][0] + n_k[2][0]*n_k[2][0] + n_k[3][0]*n_k[3][0]);
   
   n_k[0][0] = n_k[0][0]/norm;
   n_k[1][0] = n_k[1][0]/norm;
   n_k[2][0] = n_k[2][0]/norm;
   n_k[3][0] = n_k[3][0]/norm;  

   q_Osserv[0][0] = n_k[3][0];
   q_Osserv[1][0] = n_k[0][0]; 
   q_Osserv[2][0] = n_k[1][0];
   q_Osserv[3][0] = n_k[2][0];
  
}

void kalman_filtering(){
  float F[4][4], Q[4][4], H[4][4], S[4][4],F_tran[4][4],t[4][4], H_tran[4][4], t_inverse[4][4], K[4][4], c[4][4];
  float var[3], e[4][1],f[4][1],g[4][1], eye[4][4];
  
  var[0] = (0.5647/180*3.14)*(0.5647/180*3.14);
  var[1] = (0.5674/180*3.14)*(0.5647/180*3.14);
  var[2] = (0.5394/180*3.14)*(0.5647/180*3.14);
  
  initialize_Q(Q,var);  
  initialize_H(H);
  initialize_S(S);
  initialize_F(F);
  initialize_eye(eye);
 
  
  if(ab==0){
     initialize_pUpdate(p_Update);
    ab++;
  }
 // Serial.print(p_Update[3][0]);

  
  MatrixMultiply((float*)F, (float*)q_Update, 4 , 4 , 1, (float*)q_predicted);
 
  MatrixTranspose((float*)F, 4 , 4,(float*)F_tran);
  MatrixMultiply((float*)F, (float*)p_Update, 4 , 4 , 4, (float*)t);
  MatrixMultiply((float*)t, (float*)F_tran, 4 , 4 , 4, (float*)c);
  
  MatrixAddition((float*)c, (float*)Q, 4 , 4,(float*)p_predicted);

  MatrixMultiply((float*)H, (float*)p_predicted, 4 , 4 , 4, (float*)t);
  MatrixTranspose((float*)H, 4 , 4,(float*)H_tran);
  MatrixMultiply((float*)t, (float*)H_tran, 4 , 4 , 4, (float*)c);
  MatrixAddition((float*)c, (float*)S, 4 , 4,(float*)t);  
  MatrixInversion((float*)t , 4, (float*) t_inverse);
  MatrixMultiply((float*)H_tran, (float*)t_inverse, 4 , 4 , 4, (float*)t);
 
  MatrixMultiply((float*)p_predicted, (float*)t, 4 , 4 , 4, (float*)K);
  
  MatrixMultiply((float*)H, (float*)q_predicted, 4 , 4 , 1, (float*)e);
  MatrixSubtraction((float*)q_Osserv, (float*) e,  4,  1, (float*) f);
  MatrixMultiply((float*)K, (float*)f, 4 , 4 , 1, (float*)e);    
  
  MatrixAddition((float*)q_predicted, (float*)e, 4 , 1,(float*)q_Update);
  
  float norm;
    
  norm = sqrt(q_Update[0][0]*q_Update[0][0] + q_Update[1][0]*q_Update[1][0] + q_Update[2][0]*q_Update[2][0] + q_Update[3][0]*q_Update[3][0]);

  q_Update[0][0] =  q_Update[0][0]/norm;   
  q_Update[1][0] =  q_Update[1][0]/norm; 
  q_Update[2][0] =  q_Update[2][0]/norm; 
  q_Update[3][0] =  q_Update[3][0]/norm; 
  
  MatrixMultiply((float*)K, (float*)H, 4 , 4 , 4, (float*)t);
  MatrixSubtraction((float*)eye, (float*)t, 4 , 4, (float*)c);
  
  MatrixMultiply((float*)c, (float*)p_predicted, 4 , 4 , 4, (float*)p_Update);  
}

void GetAnglesFromQuaternion(){
  float q0, q1, q2, q3;

    q0=q_Update[0][0];
    q1=q_Update[1][0];
    q2=q_Update[2][0];
    q3=q_Update[3][0];
    
    Kalman_roll  = -atan2((2*(q2*q3)+2*q0*q1),(2*q0*q0+2*q3*q3-1))*180/3.14;
    Kalman_pitch = -asin(2*q1*q3-2*q0*q2)*180/3.14;
    Kalman_yaw   = -atan2(2*q1*q2+q0*q3,2*q0*q0+2*q1*q1-1)*180/3.14;
    
  myFile.print(Kalman_roll);myFile.print('\t');myFile.print(Kalman_pitch);myFile.print('\t');myFile.print(Kalman_yaw);myFile.print('\n');//Serial.print('\n');
}

void QuaternionProduct(float aa[][1], float bb[][1], float cc[][1])
{
    cc[0][0]= aa[0][0]*bb[0][0] - aa[1][0]*bb[1][0] - aa[2][0]*bb[2][0] - aa[3][0]*bb[3][0];
    cc[1][0]= aa[0][0]*bb[1][0] + aa[1][0]*bb[0][0] + aa[2][0]*bb[3][0] - aa[3][0]*bb[2][0];
    cc[2][0]= aa[0][0]*bb[2][0] - aa[1][0]*bb[3][0] + aa[2][0]*bb[0][0] + aa[3][0]*bb[1][0];
    cc[3][0]= aa[0][0]*bb[3][0] + aa[1][0]*bb[2][0] - aa[2][0]*bb[1][0] + aa[3][0]*bb[0][0];    
}

void compute_Jacobian(float J[][4],float a, float b, float c, float d, float Ax, float Ay, float Az, float Mx, float My, float Mz){
    
    J[0][0] = (2*a*Ax+2*b*Ay+2*c*Az);
    J[0][1] = (-2*b*Ax+2*a*Ay+2*d*Az);
    J[0][2] = (-2*c*Ax-2*d*Ay+2*a*Az);
    J[0][3] = (2*d*Ax-2*c*Ay+2*b*Az);

    J[1][0] = (2*b*Ax-2*a*Ay-2*d*Az);
    J[1][1] = (2*a*Ax+2*b*Ay+2*c*Az);
    J[1][2] = (2*d*Ax-2*c*Ay+2*b*Az);
    J[1][3] = (2*c*Ax+2*d*Ay-2*a*Az);

    J[2][0] = (2*c*Ax+2*d*Ay-2*a*Az);
    J[2][1] = (-2*d*Ax+2*c*Ay-2*b*Az);
    J[2][2] = (2*a*Ax+2*b*Ay+2*c*Az);
    J[2][3] = (-2*b*Ax+2*a*Ay+2*d*Az);

    J[3][0] = (2*a*Mx+2*b*My+2*c*Mz);
    J[3][1] = (-2*b*Mx+2*a*My+2*Mz*d);
    J[3][2] = (-2*c*Mx-2*d*My+2*a*Mz);
    J[3][3] = (2*d*Mx-2*c*My+2*b*Mz);

    J[4][0] = (2*b*Mx-2*a*My-2*d*Mz);
    J[4][1] = (2*a*Mx+2*b*My+2*c*Mz);
    J[4][2] = (2*d*Mx-2*c*My+2*b*Mz);
    J[4][3] = (2*c*Mx+2*d*My-2*a*Mz);

    J[5][0] = (2*c*Mx+2*d*My-2*a*Mz);
    J[5][1] = (-2*d*Mx+2*c*My-2*b*Mz);
    J[5][2] = (2*a*Mx+2*b*My+2*c*Mz);
    J[5][3] = (-2*b*Mx+2*a*My+2*d*Mz);
}

void ComputeR_Matrix(float R[][3],float a, float b, float c, float d)
{  
    R[0][0] = d*d + a*a - b*b - c*c;
    R[0][1] = 2*(a*b-c*d);
    R[0][2] = 2*(a*c+b*d);
    R[1][0] = 2*(a*b+c*d);
    R[1][1] = d*d+b*b-a*a-c*c;
    R[1][2] = 2*(b*c-a*d);
    R[2][0] = 2*(a*c-b*d);
    R[2][1] = 2*(b*c+a*d);
    R[2][2] = d*d+c*c-b*b-a*a;
}

void ComputeM_Matrix(float M[][6], float R[][3])
{  
  int i=0,j=0,n=6,m=6;
  
  for(i=0;i<n;i++){
    for(j=0;j<m;j++){
      M[i][j] =0;
    }
  }
   
  for(i=0;i<3;i++){
    for(j=0;j<3;j++){
      M[i][j] =R[i][j];
    }
  }
  
  for(i=3;i<6;i++){
    for(j=3;j<6;j++){
      M[i][j] =R[i-3][j-3];
    }
  }  
}

void Gauss_Newton_step(float n_k[][1],float y_e[][1],float y_b[][1],float J[][4],float M[][6]){
  
    float J_tran[4][6], J_product[4][4],y_bproduct[6][1], J2[4][6], n[4][1], J_inverse[4][4];
  
    MatrixTranspose((float*)J,6,4,(float*)J_tran);
    MatrixMultiply((float*)J_tran,(float*)J,4,6,4,(float*)J_product);
    MatrixInversion((float*)J_product , 4, (float*) J_inverse);
    MatrixMultiply((float*)J_inverse,(float*)J_tran,4,4,6,(float*)J2);
    MatrixMultiply((float*)M,(float*)y_b,6,6,1,(float*)y_bproduct);
   
    y_e[0][0] = y_e[0][0]- y_bproduct[0][0];
    y_e[1][0] = y_e[1][0]- y_bproduct[1][0];
    y_e[2][0] = y_e[2][0]- y_bproduct[2][0];
    y_e[3][0] = y_e[3][0]- y_bproduct[3][0];
    y_e[4][0] = y_e[4][0]- y_bproduct[4][0];
    y_e[5][0] = y_e[5][0]- y_bproduct[5][0];  
 
    MatrixMultiply((float*)J2,(float*)y_e,4,6,1,(float*)n); 
 
    n_k[0][0] =  n_k[0][0] - n[0][0];
    n_k[1][0] =  n_k[1][0] - n[1][0];
    n_k[2][0] =  n_k[2][0] - n[2][0];
    n_k[3][0] =  n_k[3][0] - n[3][0];
    
}

void initialize_Q(float Q[][4],float var[])
{

    Q[0][0] =  var[0] + var[1] + var[2];
    Q[1][0] = -var[0] + var[1] - var[2];
    Q[2][0] = -var[0] - var[1] + var[2];
    Q[3][0] =  var[0] - var[1] - var[2];

    Q[0][1] = -var[0] + var[1] - var[2];
    Q[1][1] =  var[0] + var[1] + var[2];
    Q[2][1] =  var[0] - var[1] - var[2];
    Q[3][1] = -var[0] + var[1] - var[2];

    Q[0][2] = -var[0] - var[1] + var[2];
    Q[1][2] =  var[0] - var[1] - var[2];
    Q[2][2] =  var[0] + var[1] + var[2];
    Q[3][2] = -var[0] + var[1] - var[2];

    Q[0][3] =  var[0] - var[1] - var[2];
    Q[1][3] = -var[0] - var[1] + var[2];
    Q[2][3] = -var[0] + var[1] - var[2];
    Q[3][3] =  var[0] + var[1] + var[2];
    
}

void initialize_H(float H[][4])
{
  int i,j;
  
  for(i=0;i<4;i++){
    for(j=0;j<4;j++){
      H[i][j] = 0;
  }
  }
  H[0][0] = 1;
  H[1][1] = 1;
  H[2][2] = 1;
  H[3][3] = 1;

}

void initialize_S(float S[][4]){
  int i,j;
  
  for(i=0;i<4;i++){
    for(j=0;j<4;j++){
      S[i][j] = 0;
  }
  }

  S[0][0] = 0.01;
  S[1][1] = 0.01;
  S[2][2] = 0.01;
  S[3][3] = 0.01;  
  
}

void initialize_F(float F[][4])
{
  F[0][0] = 1;
  F[1][0] = dt * gyroRate[XAXIS];
  F[2][0] = dt * gyroRate[YAXIS];
  F[3][0] = -dt * gyroRate[ZAXIS];
  
  F[0][1] = -dt * gyroRate[XAXIS];
  F[1][1] =  1;
  F[2][1] = -dt * gyroRate[ZAXIS];
  F[3][1] =  dt * gyroRate[YAXIS];

  F[0][2] = -dt * gyroRate[YAXIS];
  F[1][2] =  dt * gyroRate[ZAXIS];
  F[2][2] =  1;
  F[3][2] = -dt * gyroRate[XAXIS];
 
  F[0][3] = -dt * gyroRate[ZAXIS];
  F[1][3] = -dt * gyroRate[YAXIS];
  F[2][3] =  dt * gyroRate[XAXIS];
  F[3][3] =  1;
}

void initialize_eye(float eye[][4])
{
  int i,j;
  
  for(i=0;i<4;i++){
    for(j=0;j<4;j++){
      eye[i][j] = 0;
    }
  }
  
  eye[0][0] = 1;
  eye[1][1] = 1;
  eye[2][2] = 1;
  eye[3][3] = 1;  
}

void initialize_pUpdate(float p_Update[][4])
{
  int i,j;
  
  for(i=0;i<4;i++){
    for(j=0;j<4;j++){
      p_Update[i][j] = 0;
  }
  }
  
  p_Update[0][0] = 2;
  p_Update[1][1] = 2;
  p_Update[2][2] = 2;
  p_Update[3][3] = 2;  
}

void MatrixTranspose(float* A, int m, int n, float* C)
{
// A = input matrix (m x n)
// m = number of rows in A
// n = number of columns in A
// C = output matrix = the transpose of A (n x m)
int i, j;
for (i=0;i<m;i++)
     for(j=0;j<n;j++)
           C[m*j+i]=A[n*i+j];
}


void MatrixMultiply(float* A, float* B, int m, int p, int n, float* C)
{
// A = input matrix (m x p)
// B = input matrix (p x n)
// m = number of rows in A
// p = number of columns in A = number of rows in B
// n = number of columns in B
// C = output matrix = A*B (m x n)
int i, j, k;
for (i=0;i<m;i++)
     for(j=0;j<n;j++)
           {
           C[n*i+j]=0;
           for (k=0;k<p;k++)
                 C[n*i+j]= C[n*i+j]+A[p*i+k]*B[n*k+j];
           }
}

int MatrixInversion(float* A, int n, float* AInverse)
{
// A = input matrix (n x n)
// n = dimension of A 
// AInverse = inverted matrix (n x n)
// This function inverts a matrix based on the Gauss Jordan method.
// The function returns 1 on success, 0 on failure.
int i, j, iPass, imx, icol, irow;
float det, temp, pivot, factor;
float* ac = (float*)calloc(n*n, sizeof(float));
det = 1;
for (i = 0; i < n; i++)
     {
     for (j = 0; j < n; j++)
           {
           AInverse[n*i+j] = 0;
           ac[n*i+j] = A[n*i+j];
           }
     AInverse[n*i+i] = 1;
     }
// The current pivot row is iPass.  
// For each pass, first find the maximum element in the pivot column.
for (iPass = 0; iPass < n; iPass++)
     {
     imx = iPass;
     for (irow = iPass; irow < n; irow++)
           {
           if (fabs(A[n*irow+iPass]) > fabs(A[n*imx+iPass])) imx = irow;
           }
     // Interchange the elements of row iPass and row imx in both A and AInverse.
     if (imx != iPass)
           {
           for (icol = 0; icol < n; icol++)
                 {
                 temp = AInverse[n*iPass+icol];
                 AInverse[n*iPass+icol] = AInverse[n*imx+icol];
                 AInverse[n*imx+icol] = temp;
                 if (icol >= iPass)
                       {
                       temp = A[n*iPass+icol];
                       A[n*iPass+icol] = A[n*imx+icol];
                       A[n*imx+icol] = temp;
                       }
                 }
           }
     // The current pivot is now A[iPass][iPass].
     // The determinant is the product of the pivot elements.
     pivot = A[n*iPass+iPass];
     det = det * pivot;
     if (det == 0) 
           {
           free(ac);
           return 0;
           }
     for (icol = 0; icol < n; icol++)
           {
           // Normalize the pivot row by dividing by the pivot element.
           AInverse[n*iPass+icol] = AInverse[n*iPass+icol] / pivot;
           if (icol >= iPass) A[n*iPass+icol] = A[n*iPass+icol] / pivot;
           }
     for (irow = 0; irow < n; irow++)
           // Add a multiple of the pivot row to each row.  The multiple factor 
           // is chosen so that the element of A on the pivot column is 0.
           {
           if (irow != iPass) factor = A[n*irow+iPass];
           for (icol = 0; icol < n; icol++)
                 {
                 if (irow != iPass)
                       {
                       AInverse[n*irow+icol] -= factor * AInverse[n*iPass+icol];
                       A[n*irow+icol] -= factor * A[n*iPass+icol];
                       }
                 }
           }
     }
     free(ac);
     return 1;
}

void MatrixAddition(float* A, float* B, int m, int n, float* C)
{
// A = input matrix (m x n)
// B = input matrix (m x n)
// m = number of rows in A = number of rows in B
// n = number of columns in A = number of columns in B
// C = output matrix = A+B (m x n)
int i, j;
for (i=0;i<m;i++)
     for(j=0;j<n;j++)
           C[n*i+j]=A[n*i+j]+B[n*i+j];
}


//Matrix Subtraction Routine
void MatrixSubtraction(float* A, float* B, int m, int n, float* C)
{
// A = input matrix (m x n)
// B = input matrix (m x n)
// m = number of rows in A = number of rows in B
// n = number of columns in A = number of columns in B
// C = output matrix = A-B (m x n)
int i, j;
for (i=0;i<m;i++)
     for(j=0;j<n;j++)
           C[n*i+j]=A[n*i+j]-B[n*i+j];
}
