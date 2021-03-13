//Copyright © 2011- Kohei-tofu. All rights reserved.
//koheitech001 [at] gmail.com

//#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>
#include "KalmanFilter.h"


///
/// KalmanFilter related functions
///
_stKalmanFilter* fKalmanFilter_New(int measure, int state);
int fKalmanFilter_Initialize(_stKalmanFilter* This);
int fKalmanFilter_Delete(_stKalmanFilter* This);
int fKalman_KalmanFilter(_stKalmanFilter* This);
int fKalmanFilter_MeasurementUpdate(_stKalmanFilter* This);
int fKalmanFilter_TimeUpdate(_stKalmanFilter* This);
int fKalmanFilter_PriorEstimate(_stKalmanFilter* This);



/*!
Allocate computation space
@param    
@return   
@note     
*/
_stKalmanFilter* fKalmanFilter_New(int measure, int state)
{
	//
	_stKalmanFilter* retAddress = (_stKalmanFilter*)malloc(sizeof(_stKalmanFilter));

	retAddress->vMeasureLengh = measure;
	retAddress->vStateLength = state;

	retAddress->oMeasure = fMat_New(measure,1);//_m1
	retAddress->oH = fMat_New(measure,state);//_ms
	retAddress->oF = fMat_New(state,state);//_ss
	retAddress->oState_pre = fMat_New(state,1);//_s1
	retAddress->oErrCov_pre = fMat_New(state,state);//_ss
	//state = r�̏ꍇ
	retAddress->oCovQ = fMat_New(state,state);//_ss 
	retAddress->oG = fMat_New(state,measure);//_sm
	retAddress->oKalmanGain_cor = fMat_New(state,measure);//_sm
	retAddress->oState_cor = fMat_New(state,1);//_s1
	retAddress->oErrCov_cor = fMat_New(state,state);//_ss
	retAddress->oCovR = fMat_New(measure,measure);//_mm


	fKalmanFilter_Initialize(retAddress);

	return retAddress;
}

/*!
Initailize values on matrix 
@param    
@return   
@note     
*/
int fKalmanFilter_Initialize(_stKalmanFilter* This)
{
	int count_error = 0;
	
	//count_error += fMat_Zero(This->oCovQ);
	count_error += fMat_UnitMatrix(This->oCovQ);
	count_error += fMat_UnitMatrix(This->oG);
	count_error += fMat_UnitMatrix(This->oErrCov_pre);
	count_error += fMat_UnitMatrix(This->oH);
	count_error += fMat_UnitMatrix(This->oF);
	count_error += fMat_UnitMatrix(This->oErrCov_cor);
	
	return count_error;
}


/*!
relase memories
@param
@return
@note
*/
int fKalmanFilter_Delete(_stKalmanFilter* This)
{
	fMat_Delete(This->oMeasure);
	fMat_Delete(This->oH);
	fMat_Delete(This->oF);
	fMat_Delete(This->oState_pre);
	fMat_Delete(This->oErrCov_pre);
	fMat_Delete(This->oCovQ);
	fMat_Delete(This->oG);
	fMat_Delete(This->oKalmanGain_cor);
	fMat_Delete(This->oState_cor);
	fMat_Delete(This->oErrCov_cor);
	fMat_Delete(This->oCovR);
	

	free(This);

	return 0;
}



/*!
update kalman filter at one step
@param
@return
@note
*/
int fKalmanFilter_Run(_stKalmanFilter* This)
{
	int count_error = 0;

	count_error += fKalmanFilter_MeasurementUpdate(This);

	count_error += fKalmanFilter_TimeUpdate(This);

	return count_error;
}



/*!
Time update
@param    
@return   
@note     
*/
int fKalmanFilter_TimeUpdate(_stKalmanFilter* This)
{
	int count_error = 0;
	//int measure = This->vMeasureLengh;
	int state = This->vStateLength;


	//
	//x- = Fx^
	//
	count_error += fMat_Mlt(This->oState_pre, This->oF, This->oState_cor);


	//**
	//P- = FPFt + GQGt 
	//**

	_stMatrix* lMat_ss_1= fMat_New(state,state);//_ms

	//FP
	count_error += fMat_Mlt(This->oErrCov_pre, This->oF, This->oErrCov_cor);

	//P = (FP)Ft
	count_error += fMat_MltTrans2(This->oErrCov_pre, This->oF);//!< Dynamically allocated and rel


	//GQ
	count_error += fMat_Mlt(lMat_ss_1, This->oG, This->oCovQ);

	//GQGt
	count_error += fMat_MltTrans2(lMat_ss_1, This->oG);//!< Dynamically allocated and rel

	//P- = APAt + GQGt 
	count_error += fMat_Add2(This->oErrCov_pre, lMat_ss_1);
	
	fMat_Delete(lMat_ss_1);

	return count_error;
}

/*!
@param    
@return   
@note     x- = Fx^
*/
int fKalmanFilter_PriorEstimate(_stKalmanFilter* This)
{
	int count_error = 0;

	//
	//x- = Fx^
	//
	count_error += fMat_Mlt(This->oState_pre, This->oF, This->oState_cor);

	return count_error;
}


/*!
measurement update
@param    
@return   
@note	  
@note	  
@note	  
*/
int fKalmanFilter_MeasurementUpdate(_stKalmanFilter* This)
{
	int count_error = 0;
	int measure = This->vMeasureLengh;
	int state = This->vStateLength;

	//
	//kalman gain
	//
	_stMatrix* lPHt = fMat_New(state,measure);//_sm
	_stMatrix* lHPHtR = fMat_New(measure,measure);//_s1
	_stMatrix* linv = fMat_New(measure,measure);//_s1

	//PHt
	fMat_MltTrans(lPHt, This->oErrCov_pre, This->oH);

	//HPHt
	fMat_Mlt(lHPHtR, This->oH, lPHt);

	//HPHt+R
	fMat_Add2(lHPHtR, This->oCovR);

	//(HPHt+R)^-1
	fMat_InverseMatrix_Gauss(linv, lHPHtR);

	//KalmanGain
	fMat_Mlt(This->oKalmanGain_cor, lPHt, linv);

	fMat_Delete(lPHt);
	fMat_Delete(lHPHtR);
	fMat_Delete(linv);

	//
	//state 
	//
	_stMatrix* lHx = fMat_New(measure,1);//_m1
	_stMatrix* ly_Hx = fMat_New(measure,1);//_m1

	//Hx
	fMat_Mlt(lHx, This->oH, This->oState_pre);

	//(y-Hx)
	fMat_Sub(ly_Hx, This->oMeasure, lHx);

	//K(y-Hx)
	fMat_Mlt(This->oState_cor, This->oKalmanGain_cor, ly_Hx);

	//x + K(y-Hx)
	fMat_Add2(This->oState_cor, This->oState_pre);

	//
	fMat_Delete(lHx);
	fMat_Delete(ly_Hx);


	_stMatrix* lKH = fMat_New(state,state);//_m1
	_stMatrix* lI_KH = fMat_New(state,state);//_m1

	//KH
	fMat_Mlt(lKH, This->oKalmanGain_cor, This->oH);

	//I
	fMat_UnitMatrix(lI_KH);

	//(I-KH)
	fMat_Sub2(lI_KH, lKH);

	//(I-KH)P
	fMat_Mlt(This->oErrCov_cor, lI_KH, This->oErrCov_pre);

	fMat_Delete(lKH);
	fMat_Delete(lI_KH);

	return count_error;
}


/*!
measurement update
@param    
@return   
@note     Measurement updat using UD transformation
*/
int fKalmanFilter_MeasurementUpdate_UDTrans(_stKalmanFilter* This)
{
	int count_error = 0;

	int measure = This->vMeasureLengh;
	int state = This->vStateLength;


	//
	//KalmanGain
	//
	//
	//UD

	//HH(ms)
	//_stMatrix* U_pre = fMat_New(state,state);//_ss
	//_stMatrix* D_pre = fMat_New(state,state);//_ss

	_stMatrix* F = fMat_New(measure,state);//_ms 

	_stMatrix* f = fMat_New(state,1);//_s1
	_stMatrix* g = fMat_New(state,1);//_s1
	//_stMatrix* h = fMat_New(1,state);//_1s

	_stMatrix* u = fMat_New(state,1);//_s1
	_stMatrix* k1 = fMat_New(state,1);//_s1
	_stMatrix* gu = fMat_New(state,1);//_s1

	_stMatrix* U_cor = fMat_New(state,state);//_s1
	_stMatrix* D_cor = fMat_New(state,state);//

	_stMatrix* lH = fMat_New(measure,state);//

	_stMatrix* lP_pre = fMat_New(state,state);

	def_ElementType_mat alpha = 0;
	def_ElementType_mat alpha_ = 0;
	def_ElementType_mat lambda = 0;

	fMat_Copy(lH, This->oH);

	fMat_Copy(lP_pre, This->oErrCov_pre);

	//count_error += fMat_UD_Degradation(U_cor, D_cor, This->oErrCov_pre);
	count_error += fMat_UD_Degradation(U_cor, D_cor, lP_pre);

	count_error += fMat_Mlt(F, lH, U_cor);

	//F = Ft = (HU)t sm
	count_error += fMat_Transpose2(F);//!< Dynamically allocated
	
	for(int i = 0; i < measure; i++)
	{	
		fMat_Zero(k1);
		//fMat_Zero(U_cor);

		//f,g(H(i))

		//
		//step1
		//

		//f = colout(F) s1
		count_error += fMat_Copy_colVector(f, 0, F, i);
		//count_error += fMat_Copy_rowVector_TO_column(f, 0, F, i);

		//g s1 = ss * s1
		count_error += fMat_Mlt(g, D_cor, f);
		alpha = fMat(This->oCovR, i, i) + fMat(f, 0, 0) * fMat(g, 0, 0);

		fMat(k1, 0, 0) = fMat(g, 0, 0);

		fMat(D_cor, 0, 0) *= fMat(This->oCovR, i, i) / alpha;

		//fMat(U_cor, 0, 0) = 1;

		//count_error += fMat_Copy_colVector(U_cor, 0, u, 0);

		//
		//step2
		//
		for(int j = 1; j < state; j++)
		{
			//��(j) = ��(j-1) + f(j)g(j)
			alpha_ = alpha + fMat(f, j, 0) * fMat(g, j, 0);
		
			//d_cor(j) = d_pre(j) * ��(j-1) / ��(j)
			fMat(D_cor, j, j) *= alpha / alpha_;

			//��(j) = f(j) / ��(j-1)
			lambda = fMat(f, j, 0) / alpha;

			//
			//u_cor(j) = u_pre(j) - ��(j) * K(j-1)
			//
			count_error += fMat_Multiplier_colVector(gu, 0, U_cor, j, fMat(g, j, 0));

			count_error += fMat_Multiplier_colVector(u, 0, k1, 0, -lambda);
			//count_error += fMat_Multiplier_colVector(U_cor, j, k1, 0, -lambda);
		
			count_error += fMat_Add2_colVector(U_cor, j, u, 0);

			//
			//K(j) = K(j-1) + g(j) * u_pre(j)
			//
			//Kj = Kj-1 + gu
			count_error += fMat_Add2_colVector(k1, 0, gu, 0);
			//fMat_Add2_colVector(k1, 0, U_pre, j);

			alpha = alpha_;
		}

		//
		//step3
		//
		count_error += fMat_Multiplier_colVector(This->oKalmanGain_cor, i, k1, 0, (1 / alpha_));
	}
	//
	//P = U D Ut
	//


	//P = (UD)Ut
	count_error += fMat_MltTrans2(This->oErrCov_cor, U_cor);//!< Dynamically allocated

	//count_error += fMat_Check_SymmetricMatrix(This->oErrCov_cor);

	fMat_Delete(F);
	fMat_Delete(f);
	fMat_Delete(g);
	//fMat_Delete(h);
	fMat_Delete(u);
	fMat_Delete(k1);
	fMat_Delete(gu);
	fMat_Delete(U_cor);
	fMat_Delete(D_cor);
	fMat_Delete(lP_pre);

	//
	//estimate
	//
	_stMatrix* lMat_m1_1 = fMat_New(measure,1);//_m1
	_stMatrix* lMat_m1_2 = fMat_New(measure,1);//_m1

	_stMatrix* lMat_s1_1 = fMat_New(state,1);//_m1

	for(int i = 0; i < measure; i++)
	{
		//fMat_Set(*(input + i), i, 0, lMat_m1_1);
		fMat(lMat_m1_1, i, 0) = fMat(This->oMeasure, i, 0);//*(input + i);
	}

	//Hx m1 = ms * s1
	count_error += fMat_Mlt(lMat_m1_2, lH, This->oState_pre);

	//(z - Hx) m1
	count_error += fMat_Sub2(lMat_m1_1, lMat_m1_2);

	//K(z-Hx) s1 = sm * m1
	count_error += fMat_Mlt(lMat_s1_1, This->oKalmanGain_cor, lMat_m1_1);

	//estimate
	//x^ = x- + K(z-Hx)
	count_error += fMat_Add(This->oState_cor, This->oState_pre, lMat_s1_1);

	fMat_Delete(lMat_m1_1);
	fMat_Delete(lMat_m1_2);
	fMat_Delete(lMat_s1_1);

	return count_error;
}




/*!
measurement update
@param    
@return   
@note	  There are 2 options
@note     UD transformation and Sqrt transformation  
*/
int fKalmanFilter_MeasurementUpdate_SqrTrans(_stKalmanFilter* This)
{
	int count_error = 0;

	int measure = This->vMeasureLengh;
	int state = This->vStateLength;



	//
	//KalmanGain
	//
	//
	//UD

	//HH(ms)
	//_stMatrix* U_pre = fMat_New(state,state);//_ss
	//_stMatrix* D_pre = fMat_New(state,state);//_ss

	_stMatrix* F = fMat_New(measure,state);//_ms 

	_stMatrix* f = fMat_New(state,1);//_s1
	_stMatrix* g = fMat_New(state,1);//_s1
	//_stMatrix* h = fMat_New(1,state);//_1s

	_stMatrix* u = fMat_New(state,1);//_s1
	_stMatrix* k1 = fMat_New(state,1);//_s1
	_stMatrix* gu = fMat_New(state,1);//_s1

	_stMatrix* U_cor = fMat_New(state,state);//_s1
	_stMatrix* D_cor = fMat_New(state,state);//

	_stMatrix* lH = fMat_New(measure,state);//

	_stMatrix* lP_pre = fMat_New(state,state);

	def_ElementType_mat alpha = 0;
	def_ElementType_mat alpha_ = 0;
	def_ElementType_mat lambda = 0;

	_stMatrix* lRCopy = fMat_New(measure, measure);
	//_stMatrix* lSqrMatR = fMat_New(measure,measure);
	_stMatrix* lSqrMatR_inv = fMat_New(measure,measure);

	fMat_Copy(lRCopy, This->oCovR);

	//R^1/2
	fMat_CholeskyDecomposition(lSqrMatR_inv, lRCopy);
	//fMat_CholeskyDecomposition(lSqrMatR_inv, This->oCovR);

	//R^-1/2
	//fMat_InverseMatrix_Gauss(lSqrMatR_inv, lSqrMatR);
	fMat_InverseMatrix_Gauss2(lSqrMatR_inv);//!< Dynamically allocated

	//H:=R^-1/2 H
	count_error += fMat_Mlt(lH, lSqrMatR_inv, This->oH);


	fMat_Copy(lP_pre, This->oErrCov_pre);

	//count_error += fMat_UD_Degradation(U_cor, D_cor, This->oErrCov_pre);
	count_error += fMat_UD_Degradation(U_cor, D_cor, lP_pre);

	count_error += fMat_Mlt(F, lH, U_cor);

	//F = Ft = (HU)t sm
	count_error += fMat_Transpose2(F);//!< Dynamically allocated
	
	for(int i = 0; i < measure; i++)
	{	
		fMat_Zero(k1);
		//fMat_Zero(U_cor);

		//f,g(H(i))

		//
		//step1
		//

		//f = colout(F) s1
		count_error += fMat_Copy_colVector(f, 0, F, i);
		//count_error += fMat_Copy_rowVector_TO_column(f, 0, F, i);

		//g s1 = ss * s1
		count_error += fMat_Mlt(g, D_cor, f);
		alpha = 1 + fMat(f, 0, 0) * fMat(g, 0, 0);

		fMat(k1, 0, 0) = fMat(g, 0, 0);

		fMat(D_cor, 0, 0) *= 1 / alpha;

		//
		//step2
		//
		for(int j = 1; j < state; j++)
		{
			//��(j) = ��(j-1) + f(j)g(j)
			alpha_ = alpha + fMat(f, j, 0) * fMat(g, j, 0);
		
			//d_cor(j) = d_pre(j) * ��(j-1) / ��(j)
			fMat(D_cor, j, j) *= alpha / alpha_;

			//��(j) = f(j) / ��(j-1)
			lambda = fMat(f, j, 0) / alpha;

			//
			//u_cor(j) = u_pre(j) - ��(j) * K(j-1)
			//
			count_error += fMat_Multiplier_colVector(gu, 0, U_cor, j, fMat(g, j, 0));

			count_error += fMat_Multiplier_colVector(u, 0, k1, 0, -lambda);
			//count_error += fMat_Multiplier_colVector(U_cor, j, k1, 0, -lambda);
		
			count_error += fMat_Add2_colVector(U_cor, j, u, 0);

			//
			//K(j) = K(j-1) + g(j) * u_pre(j)
			//
			//Kj = Kj-1 + gu
			count_error += fMat_Add2_colVector(k1, 0, gu, 0);
			//fMat_Add2_colVector(k1, 0, U_pre, j);

			alpha = alpha_;
		}

		//
		//step3
		//
		count_error += fMat_Multiplier_colVector(This->oKalmanGain_cor, i, k1, 0, (1 / alpha_));
	}
	//
	//P = U D Ut
	//


	//P = (UD)Ut
	count_error += fMat_MltTrans2(This->oErrCov_cor, U_cor);//!< Dynamically allocated

	//count_error += fMat_Check_SymmetricMatrix(This->oErrCov_cor);

	fMat_Delete(F);
	fMat_Delete(f);
	fMat_Delete(g);
	//fMat_Delete(h);
	fMat_Delete(u);
	fMat_Delete(k1);
	fMat_Delete(gu);
	fMat_Delete(U_cor);
	fMat_Delete(D_cor);
	fMat_Delete(lP_pre);

	//
	//estimate
	//
	_stMatrix* lMat_m1_1 = fMat_New(measure,1);//_m1
	_stMatrix* lMat_m1_2 = fMat_New(measure,1);//_m1

	_stMatrix* lMat_s1_1 = fMat_New(state,1);//_m1

	//y:=R^(-1/2) y
	count_error += fMat_Mlt(lMat_m1_1, lSqrMatR_inv, This->oMeasure);


	//Hx m1 = ms * s1
	count_error += fMat_Mlt(lMat_m1_2, lH, This->oState_pre);

	//(z - Hx) m1
	count_error += fMat_Sub2(lMat_m1_1, lMat_m1_2);

	//K(z-Hx) s1 = sm * m1
	count_error += fMat_Mlt(lMat_s1_1, This->oKalmanGain_cor, lMat_m1_1);

	//estimate
	//x^ = x- + K(z-Hx)
	count_error += fMat_Add(This->oState_cor, This->oState_pre, lMat_s1_1);


	fMat_Delete(lMat_m1_1);
	fMat_Delete(lMat_m1_2);
	fMat_Delete(lMat_s1_1);

	fMat_Delete(lRCopy);
	//fMat_Delete(lSqrMatR);
	fMat_Delete(lSqrMatR_inv);

	return count_error;
}


