/**
  ****************************(C) COPYRIGHT 2023 TJU****************************
  * @file       6dof_kinematic.c/h
  * @brief      Kinematics solution of robotic arm,
  *             ��е���˶�ѧ����
  * @note       
  * @history
  *  Version    Date            Author          Modificxtion
  *  V1.0.0     Nov-20-2023     Wang Yiwen       1. done
  *  V1.0.1     Dec-1-2023      Wang Yiwen       1. add quaterniont transform
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 TJU****************************
  */
#include "6dof_kinematic.h"
#include "cmsis_os.h"

float cosf(float x)
{
    return arm_cos_f32(x);
}

float sinf(float x)
{
    return arm_sin_f32(x);
}


//Z-Y-Xŷ����->��ת����
static void EulerAngleToRotMat(const float* _eulerAngles, float* _rotationM)//Z-Y-X
{
    float cx, cy, cz, sx, sy, sz;

    cz = cosf(_eulerAngles[0]);
    cy = cosf(_eulerAngles[1]);
    cx = cosf(_eulerAngles[2]);
    sz = sinf(_eulerAngles[0]);
    sy = sinf(_eulerAngles[1]);
    sx = sinf(_eulerAngles[2]);

    _rotationM[0] = cy * cz;
    _rotationM[1] = -cx * sz + sx * sy * cz;
    _rotationM[2] =  sx * sz + cx * sy * cz;
    _rotationM[3] = cy * sz;
    _rotationM[4] = sx * sy * sz + cx * cz;
    _rotationM[5] = cx * sy * sz - sx * cz;
    _rotationM[6] = -sy;
    _rotationM[7] = cy * sx;
    _rotationM[8] = cx * cy;
}


//��ת����->Z-Y-Xŷ����
static void RotMatToEulerAngle(const float* _rotationM, float* _eulerAngles)//Z-Y-X
{
    float Y, P, R;
		Y = atan2f(_rotationM[3], _rotationM[0]);
		P = atan2f(-_rotationM[6], sqrtf(_rotationM[7] * _rotationM[7] + _rotationM[8] * _rotationM[8]));
		R = atan2f(_rotationM[7], _rotationM[8]);
    
    _eulerAngles[0] = Y;
    _eulerAngles[1] = P;
    _eulerAngles[2] = R;
}


//��Ԫ��->��ת����
static void QuaToRotMat(const float *_Q, float* _R)
{
  _R[0] = 1 - 2 * (_Q[2] * _Q[2]) - 2 * (_Q[3] * _Q[3]);
  _R[1] = 2 * _Q[1] * _Q[2] - 2 * _Q[0] * _Q[3];
  _R[2] = 2 * _Q[1] * _Q[3] + 2 * _Q[0] * _Q[2];
  _R[3] = 2 * _Q[1] * _Q[2] + 2 * _Q[0] * _Q[3];
  _R[4] = 1 - 2 * (_Q[1] * _Q[1]) - 2 * (_Q[3] * _Q[3]);
  _R[5] = 2 * _Q[2] * _Q[3] - 2 * _Q[0] * _Q[1];
  _R[6] = 2 * _Q[1] * _Q[3] - 2 * _Q[0] * _Q[2];
  _R[7] = 2 * _Q[2] * _Q[3] + 2 * _Q[0] * _Q[1];
  _R[8] = 1 - 2 * (_Q[1] * _Q[1]) - 2 * (_Q[2] * _Q[2]);
}


//  _R[0] = (0,0);
//  _R[1] = (0,1);
//  _R[2] = (0,2);
//  _R[3] = (1,0);
//  _R[4] = (1,1);
//  _R[5] = (1,2);
//  _R[6] = (2,0);
//  _R[7] = (2,1);
//  _R[8] = (2,2);

//��ת����->��Ԫ��
void RotMatToQua(const float* _R, float *_Q)
{
		float trace = _R[0] + _R[4] + _R[8];
	
    if (trace > 0.0f) 
    {
        float s = sqrt(trace + 1.0f);
        _Q[0] = (s * 0.5f);
        s = 0.5f / s;
        _Q[1] = ((_R[7] - _R[5]) * s);
        _Q[2] = ((_R[2] - _R[6]) * s);
        _Q[3] = ((_R[3] - _R[1]) * s);
    } 
    
    else 
    {
        int i = _R[0] < _R[4] ? (_R[4] < _R[8] ? 2 : 1) : (_R[0] < _R[8] ? 2 : 0); 
        int j = (i + 1) % 3;  
        int k = (i + 2) % 3;

        float s = sqrt(_R[i * 3 + i] - _R[j * 3 + j]  - _R[k * 3 + k] + 1.0f);
        _Q[i + 1] = s * 0.5f;
        s = 0.5f / s;

        _Q[0] 		= (_R[k * 3 + j] - _R[j * 3 + k]) * s;
        _Q[j + 1] = (_R[j * 3 + i] + _R[i * 3 + j]) * s;
        _Q[k + 1] = (_R[k * 3 + i] + _R[i * 3 + k]) * s;
    }
}

//��Ԫ��->Z-Y-Xŷ����
void QuaToEulerAngle(const float *_Q, float* _eulerAngles)
{
	float _R[9];
	float _rotationM[9];
  _R[0] = 1 - 2 * (_Q[2] * _Q[2]) - 2 * (_Q[3] * _Q[3]);
  _R[1] = 2 * _Q[1] * _Q[2] - 2 * _Q[0] * _Q[3];
  _R[2] = 2 * _Q[1] * _Q[3] + 2 * _Q[0] * _Q[2];
  _R[3] = 2 * _Q[1] * _Q[2] + 2 * _Q[0] * _Q[3];
  _R[4] = 1 - 2 * (_Q[1] * _Q[1]) - 2 * (_Q[3] * _Q[3]);
  _R[5] = 2 * _Q[2] * _Q[3] - 2 * _Q[0] * _Q[1];
  _R[6] = 2 * _Q[1] * _Q[3] - 2 * _Q[0] * _Q[2];
  _R[7] = 2 * _Q[2] * _Q[3] + 2 * _Q[0] * _Q[1];
  _R[8] = 1 - 2 * (_Q[1] * _Q[1]) - 2 * (_Q[2] * _Q[2]);
	
	_rotationM[0] = _R[0];
	_rotationM[3] = _R[1];
	_rotationM[6] = _R[2];
	_rotationM[1] = _R[3];
	_rotationM[4] = _R[4];
	_rotationM[7] = _R[5];
	_rotationM[2] = _R[6];
	_rotationM[5] = _R[7];
	_rotationM[8] = _R[8];
	
	float Y, P, R;
	Y = atan2f(_rotationM[3], _rotationM[0]);
	P = atan2f(-_rotationM[6], sqrtf(_rotationM[7] * _rotationM[7] + _rotationM[8] * _rotationM[8]));
	R = atan2f(_rotationM[7], _rotationM[8]);
	
	_eulerAngles[0] = Y;
	_eulerAngles[1] = P;
	_eulerAngles[2] = R;
}

//Z-Y-Xŷ����->��Ԫ��
void EulerAngleToQua(const float* _eulerAngles, float *_Q)
{
	float cx, cy, cz, sx, sy, sz;
	float _rotationM[9];
	float _R[9];

	cz = cosf(_eulerAngles[0]);
	cy = cosf(_eulerAngles[1]);
	cx = cosf(_eulerAngles[2]);
	sz = sinf(_eulerAngles[0]);
	sy = sinf(_eulerAngles[1]);
	sx = sinf(_eulerAngles[2]);

	_rotationM[0] = cy * cz;
	_rotationM[1] = -cx * sz + sx * sy * cz;
	_rotationM[2] =  sx * sz + cx * sy * cz;
	_rotationM[3] = cy * sz;
	_rotationM[4] = sx * sy * sz + cx * cz;
	_rotationM[5] = cx * sy * sz - sx * cz;
	_rotationM[6] = -sy;
	_rotationM[7] = cy * sx;
	_rotationM[8] = cx * cy;

	_R[0] = _rotationM[0];
	_R[3] = _rotationM[1];
	_R[6] = _rotationM[2];
	_R[1] = _rotationM[3];
	_R[4] = _rotationM[4];
	_R[7] = _rotationM[5];
	_R[2] = _rotationM[6];
	_R[5] = _rotationM[7];
	_R[8] = _rotationM[8];
	
	float trace = _R[0] + _R[4] + _R[8];
	if (trace > 0.0f) 
	{
			float s = sqrt(trace + 1.0f);
			_Q[0] = (s * 0.5f);
			s = 0.5f / s;
			_Q[1] = ((_R[7] - _R[5]) * s);
			_Q[2] = ((_R[2] - _R[6]) * s);
			_Q[3] = ((_R[3] - _R[1]) * s);
	} 
	
	else 
	{
			int i = _R[0] < _R[4] ? (_R[4] < _R[8] ? 2 : 1) : (_R[0] < _R[8] ? 2 : 0); 
			int j = (i + 1) % 3;  
			int k = (i + 2) % 3;

			float s = sqrt(_R[i * 3 + i] - _R[j * 3 + j]  - _R[k * 3 + k] + 1.0f);
			_Q[i + 1] = s * 0.5f;
			s = 0.5f / s;

			_Q[0] 		= (_R[k * 3 + j] - _R[j * 3 + k]) * s;
			_Q[j + 1] = (_R[j * 3 + i] + _R[i * 3 + j]) * s;
			_Q[k + 1] = (_R[k * 3 + i] + _R[i * 3 + k]) * s;
	}
}

//��е������ʼ��
void Joint6D_Init(Robotic_6DOF * Robotic_6D)
{
		//alpha,a,d,theta
    float DH_Matrix[5][4] = {
        {0.0f,          0.0f,        364.0f,       0.0f},
        {0.0f,         260.0f,        0.0f,       0.0f},
        {0,        	0.0f,        235.0f,       0.0f},
        {PI/2,         81.0f,     0.0f,       0.0f},
        {-PI/2,         0.0f,     170.0f,       0.0f}};
		
		//����DH����->Robotic_6D�ṹ��
		for(int i = 0; i < 5; i++)
		{
				Robotic_6D->_alpha[i]=	DH_Matrix[i][0];
				Robotic_6D->_a[i]=	DH_Matrix[i][1];
				Robotic_6D->_d[i]=	DH_Matrix[i][2];
		}

    //��ʼ��λ�˱任����T
		for(int i = 0; i < 5; i++)
		{
		    Robotic_6D->_T_data[i] = (float *)user_malloc(sizeof(float) * 4 * 4);
				memset(Robotic_6D->_T_data[i], 0, sizeof(float) * 4 * 4);
			  Matrix_Init(&Robotic_6D->_T[i], 4, 4, (float *)Robotic_6D->_T_data[i]);
		}
		
		//��ʼ��������temp����
		for(int i = 0; i < 2; i++)
		{
				Robotic_6D->fk_temp_matrix_data[i] = (float *)user_malloc(sizeof(float) * 4 * 4);
				memset(Robotic_6D->fk_temp_matrix_data[i], 0, sizeof(float) * 4 * 4);
				Matrix_Init(&Robotic_6D->fk_temp_matrix[i], 4, 4, (float *)Robotic_6D->fk_temp_matrix_data[i]);		
		}
				
		//��ʼ��R02��ת����
		Robotic_6D->_R02_inv_data = (float *)user_malloc(sizeof(float) * 3 * 3);
		memset(Robotic_6D->_R02_inv_data, 0, sizeof(float) * 3 * 3);
		Matrix_Init(&Robotic_6D->R02_inv, 3, 3, (float *)Robotic_6D->_R02_inv_data);


		//��ʼ��R25��ת����
		Robotic_6D->_R25_data = (float *)user_malloc(sizeof(float) * 3 * 3);
		memset(Robotic_6D->_R25_data, 0, sizeof(float) * 3 * 3);
		Matrix_Init(&Robotic_6D->_R25, 3, 3, (float *)Robotic_6D->_R25_data);
		
		//��ʼ��R06��ת����
		Robotic_6D->_R05_data = (float *)user_malloc(sizeof(float) * 3 * 3);
		memset(Robotic_6D->_R05_data, 0, sizeof(float) * 3 * 3);
		Matrix_Init(&Robotic_6D->_R05, 3, 3, (float *)Robotic_6D->_R05_data);
		
		//��ʼ�������temp����		
		Robotic_6D->_ik_temp_matrix_data = (float *)user_malloc(sizeof(float) * 3 * 3);
		memset(Robotic_6D->_ik_temp_matrix_data, 0, sizeof(float) * 3 * 3);
		Matrix_Init(&Robotic_6D->_ik_temp_matrix, 3, 3, (float *)Robotic_6D->_ik_temp_matrix_data);	

		//��ʼ��P56��temp����
		Robotic_6D->_P45_data = (float *)user_malloc(sizeof(float) * 3 * 1);
		memset(Robotic_6D->_P45_data, 0, sizeof(float) * 3 * 1);
		Matrix_Init(&Robotic_6D->_P45, 3, 1, (float *)Robotic_6D->_P45_data);	
		Robotic_6D->_P45_temp_data = (float *)user_malloc(sizeof(float) * 3 * 1);
		memset(Robotic_6D->_P45_temp_data, 0, sizeof(float) * 3 * 1);
		Matrix_Init(&Robotic_6D->_P45_temp, 3, 1, (float *)Robotic_6D->_P45_temp_data);		
}


//��е�������˶�ѧ���
void SolveFK(Robotic_6DOF * Robotic_6D, const Joint6D_t *_Joint6D, Pose6D_t *_Pose6D)
{
    float cosq, sinq;
    float cosx, sina;	
		float R05[9];
		float EulerAngles[3];
		float Quaterniont[4];
	
		//����ֵ
    for (int i = 0; i < 5; i++)
    {
        cosx = cosf(Robotic_6D->_alpha[i]);
        sina = sinf(Robotic_6D->_alpha[i]);
        cosq = cosf(_Joint6D->theta[i]);
        sinq = sinf(_Joint6D->theta[i]);
			
				Robotic_6D->_T_data[i][0] = cosq;
        Robotic_6D->_T_data[i][1] = -sinq;
        Robotic_6D->_T_data[i][2] = 0.0f;
        Robotic_6D->_T_data[i][3] = Robotic_6D->_a[i];
        Robotic_6D->_T_data[i][4] =  cosx * sinq;
        Robotic_6D->_T_data[i][5] =  cosx * cosq;
        Robotic_6D->_T_data[i][6] = -sina;
        Robotic_6D->_T_data[i][7] = -sina * Robotic_6D->_d[i];
        Robotic_6D->_T_data[i][8] =  sina * sinq;
			  Robotic_6D->_T_data[i][9] =  sina * cosq;
			  Robotic_6D->_T_data[i][10] = cosx;
			  Robotic_6D->_T_data[i][11] = cosx * Robotic_6D->_d[i];
			  Robotic_6D->_T_data[i][12] = 0.0f;
			  Robotic_6D->_T_data[i][13] = 0.0f;
			  Robotic_6D->_T_data[i][14] = 0.0f;
			  Robotic_6D->_T_data[i][15] = 1.0f;
    }	
			
		//����任�������
		Robotic_6D->MatStatus = Matrix_Multiply(&Robotic_6D->_T[0], &Robotic_6D->_T[1], &Robotic_6D->fk_temp_matrix[0]);		
		Robotic_6D->MatStatus = Matrix_Multiply(&Robotic_6D->fk_temp_matrix[0], &Robotic_6D->_T[2], &Robotic_6D->fk_temp_matrix[1]);
		Robotic_6D->MatStatus = Matrix_Multiply(&Robotic_6D->fk_temp_matrix[1], &Robotic_6D->_T[3], &Robotic_6D->fk_temp_matrix[0]);
		Robotic_6D->MatStatus = Matrix_Multiply(&Robotic_6D->fk_temp_matrix[0], &Robotic_6D->_T[4], &Robotic_6D->fk_temp_matrix[1]);

		
		//ȡ����任�����е���ת����
		for(int i = 0; i < 3; i++)
		{
				for(int j = 0; j < 3; j++)
				{
						R05[i * 3 + j] = Robotic_6D->fk_temp_matrix_data[1][i * 4 + j];
				}
		}

		//��ת����->ĩ��ŷ����Z-Y-X
		RotMatToEulerAngle(R05, &(EulerAngles[0]));
		//��ת����->��Ԫ��
		RotMatToQua(R05, &(Quaterniont[0]));
		
		_Pose6D->X = Robotic_6D->fk_temp_matrix_data[1][3];
		_Pose6D->Y = Robotic_6D->fk_temp_matrix_data[1][7];
		_Pose6D->Z = Robotic_6D->fk_temp_matrix_data[1][11];
		_Pose6D->Yaw = EulerAngles[0] * RAD_TO_DEG;//yaw
		_Pose6D->Pitch = EulerAngles[1] * RAD_TO_DEG;//pitch
		_Pose6D->Roll = EulerAngles[2] * RAD_TO_DEG;//roll	
		_Pose6D->Q[0] = Quaterniont[0];
		_Pose6D->Q[1] = Quaterniont[1];
		_Pose6D->Q[2] = Quaterniont[2];
		_Pose6D->Q[3] = Quaterniont[3];
}

float R05[9];
float xyz[3];
//��е�������˶�ѧ���
bool SolveIK(Robotic_6DOF * Robotic_6D, Pose6D_t *_inputPose6D, Solver6D_t *_Out_Solver6D, const Joint6D_t *_lastJoint6D, uint8_t _Quaterniont_mode)
{
    float q12[2][2];//theta1,theta2
    float q345[2][3];//theta3,theta4,theta5
		float x,y,z;
		float q0, a0,temp;
		float tmp;//�м����
		float Euler[3];
		float Quaterniont[4];
		int ind_theta4=2;//��������
		
		//����ĩ��λ�������ת����
		if(_Quaterniont_mode)//��Ԫ��ģʽ���Ӿ�ģʽ��
		{
				//��Ԫ��->��ת����
				QuaToRotMat(&(_inputPose6D->Q[0]), Robotic_6D->_R05_data);
				for(int i = 0; i < 3; i++)
				//ȡ��ת����
				{
						for(int j = 0; j < 3; j++)
						{
								R05[i * 3 + j] = Robotic_6D->_R05_data[i * 3 + j];
						}
				}
				//��ת����->ĩ��ŷ����Z-Y-X
				RotMatToEulerAngle(R05, &(Euler[0]));
				//Ŀ��ĩ����̬��ŷ���Ǹ���
				_inputPose6D->Yaw = Euler[0] * RAD_TO_DEG;//yaw
				_inputPose6D->Pitch = Euler[1] * RAD_TO_DEG;//pitch
				_inputPose6D->Roll = Euler[2] * RAD_TO_DEG;//roll					
		}
		else//ŷ����ģʽ
		{
				Euler[0]=_inputPose6D->Yaw/RAD_TO_DEG;
				Euler[1]=_inputPose6D->Pitch/RAD_TO_DEG;
				Euler[2]=_inputPose6D->Roll/RAD_TO_DEG;
				//ŷ����->��ת����
				EulerAngleToRotMat(&(Euler[0]), Robotic_6D->_R05_data);

				//ȡ��ת����
				for(int i = 0; i < 3; i++)
				{
						for(int j = 0; j < 3; j++)
						{
								R05[i * 3 + j] = Robotic_6D->_R05_data[i * 3 + j];
						}
				}
				//��ת����->��Ԫ��
				RotMatToQua(R05, &(Quaterniont[0]));
				//Ŀ��ĩ����̬����Ԫ������
				_inputPose6D->Q[0] = Quaterniont[0];
				_inputPose6D->Q[1] = Quaterniont[1];
				_inputPose6D->Q[2] = Quaterniont[2];
				_inputPose6D->Q[3] = Quaterniont[3];			
		}
	
		//�ϴν������֪�Ƕȸ�ֵ������ʵû���ã�
		for (int i = 0; i < 2; i++)
    {
        q12[i][0]  = _lastJoint6D->theta[0];
        q12[i][1]  = _lastJoint6D->theta[1];
        q345[i][0] = _lastJoint6D->theta[2];
        q345[i][1] = _lastJoint6D->theta[3];
        q345[i][2] = _lastJoint6D->theta[4];
    }
		
		//����0�����ؽ�3��4��5��ԭ������
		Robotic_6D->_P45_data[0] = 170.0f;
		Robotic_6D->_P45_data[1] = 0.0f;		
		Robotic_6D->_P45_data[2] = 0.0f;
		Robotic_6D->MatStatus = Matrix_Multiply(&Robotic_6D->_R05, &Robotic_6D->_P45, &Robotic_6D->_P45_temp);
		
		//��ֵx,y,z
//		x =	_inputPose6D->X - Robotic_6D->_P45_temp_data[0];
//		y = _inputPose6D->Y - Robotic_6D->_P45_temp_data[1];
//		z = _inputPose6D->Z - Robotic_6D->_P45_temp_data[2] - Robotic_6D->_d[0];//֮���һ�ɻ����߶ȣ���������������
//		_Out_Solver6D->high_position = z;
//		Robotic_6D->high = Robotic_6D->high
		x =	_inputPose6D->X ;
		y = _inputPose6D->Y ;
		z = _inputPose6D->Z - Robotic_6D->_d[0];//֮���һ�ɻ����߶ȣ���������������
		_Out_Solver6D->high_position = z+1.0f;
		xyz [0] = x;
		xyz [1] = y;
		xyz [2] = z;
		
		//����1�����theta1��theta2
		q0 = atan2f(y,x);                                   
		arm_sqrt_f32(x*x + y*y , &a0);
		temp = (x*x + y*y + Robotic_6D->_a[1]*Robotic_6D->_a[1] - Robotic_6D->_d[2]*Robotic_6D->_d[2])/(2*a0*Robotic_6D->_a[1]);
		if(fabs(temp)<1)
		{
		q12[0][0] = -acos(temp) + q0;
		q12[1][0] =  acos(temp) + q0;
		}
		temp = (x*x + y*y - Robotic_6D->_a[1]*Robotic_6D->_a[1] + Robotic_6D->_d[2]*Robotic_6D->_d[2])/(2*a0*Robotic_6D->_d[2]);
		if(fabs(temp)< 1)
		{
		q12[0][1] =  acos(temp) + q0 - q12[0][0];
		q12[1][1] = -acos(temp) + q0 - q12[1][0];
		}
		
		//����2�����theta3,4,5
		for(int i = 0; i < 2; i++)
		{
			q0 = q12[i][0] + q12[i][1];
			Robotic_6D->_R02_inv_data[0] =  cosf(q0);
			Robotic_6D->_R02_inv_data[1] =  sinf(q0);
			Robotic_6D->_R02_inv_data[2] =  0;
			Robotic_6D->_R02_inv_data[3] =  -sinf(q0);
			Robotic_6D->_R02_inv_data[4] =  cosf(q0);
			Robotic_6D->_R02_inv_data[5] =  0;
			Robotic_6D->_R02_inv_data[6] =  0;
			Robotic_6D->_R02_inv_data[7] =  0;
			Robotic_6D->_R02_inv_data[8] =  1;
			
			Robotic_6D->MatStatus = Matrix_Multiply(&Robotic_6D->R02_inv, &Robotic_6D->_R05, &Robotic_6D->_R25);
			arm_sqrt_f32(Robotic_6D->_R25_data[2] * Robotic_6D->_R25_data[2] + Robotic_6D->_R25_data[5] * Robotic_6D->_R25_data[5], &tmp);											
						
			//theta4=0
			if(tmp < 0.00001f)
			{
					//����⣺theta4=0/theta4=180������һ�鲻����Լ����Χ�����ڱ���������Ϊ�ǵ���
					for(int k = 0; k < ind_theta4; k++)
					{
							float s3,s5;
							float c3,c5;									
							//theta4
							q345[k][1] = k * PI;									
							//theta3,��theta4�ӽ�0��ʱֻ�����theta3��theta5�ĺͻ�����theta3����һ�εĽǶȣ�������0��
							q345[k][0] = _lastJoint6D->theta[2];
							//theta5
							c3 = cosf(_lastJoint6D->theta[2]);
							s3 = sinf(_lastJoint6D->theta[2]);												
							s5 =  c3 * Robotic_6D->_R25_data[3] - s3 * Robotic_6D->_R25_data[0];
							c5 =  c3 * Robotic_6D->_R25_data[4] - s3 * Robotic_6D->_R25_data[1] ;										
							q345[k][2] = atan2f(s5,c5);
							//����
					}
					}
			else
			{		
							//˫��			
					for(int k = 0; k < ind_theta4; k++)
					{
							//theta5
							q345[k][1] = atan2f((2 * k - 1) * tmp, Robotic_6D->_R25_data[8]);
							//theta4,theta6
							float sinq4 = sinf(q345[k][1]);
							q345[k][0] = atan2f(-Robotic_6D->_R25_data[5] / sinq4, -Robotic_6D->_R25_data[2] / sinq4);
							q345[k][2] = atan2f( Robotic_6D->_R25_data[7] / sinq4,  Robotic_6D->_R25_data[6] / sinq4);
							//˫��
					}
			}								
//			
//			
//			//��ֵ
			for(int k = 0; k < ind_theta4; k++)
			{
					//theta1��ֵ 
					if(q12[i][0] > PI)
					{
							_Out_Solver6D->theta[2 * i + k][0] = q12[i][0] - 2 * PI;
					}	
					else if(q12[i][0] < -PI)
					{
							_Out_Solver6D->theta[2 * i + k][0] = q12[i][0] + 2 * PI;
					}
					else
					{
							_Out_Solver6D->theta[2 * i + k][0] = q12[i][0];
					}
					//theta2��ֵ					
					if(q12[i][1] > PI)
					{
							_Out_Solver6D->theta[2 * i + k][1] = q12[i][1] - 2 * PI;
					}	
					else if(q12[i][0] < -PI)
					{
							_Out_Solver6D->theta[2 * i + k][1] = q12[i][1] + 2 * PI;
					}
					else
					{
							_Out_Solver6D->theta[2 * i + k][1] = q12[i][1];
					}
					
					
					//theta3,theta4,theta5��ֵ
					for(int n = 0; n < 3; n++)
					{
							if(q345[k][n] > PI)
							{
									_Out_Solver6D->theta[2 * i + k][2 + n] = q345[k][n] - 2 * PI;
							}	
							else if(q345[k][n] < -PI)
							{
									_Out_Solver6D->theta[2 * i + k][2 + n] = q345[k][n] + 2 * PI;
							}
							else
							{
									_Out_Solver6D->theta[2 * i + k][2 + n] = q345[k][n];
							}		
					}				
			}
		}
//		osDelay(1);
		
		
		return true;
}

