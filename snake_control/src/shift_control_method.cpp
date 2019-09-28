/*
 * shift_control_method.cpp
 *
 *  Created on: Feb 22, 2017
 *      Author: QI
 */
#include "shift_control_method.h"

void ShiftControlMethod::Init(RobotSpec spec)
{
	int NUM_JOINT = spec.num_joint();
	hold_data.shift_param.resize(NUM_JOINT);

	int max_hold_num = 28;     	 //角度を保持する数 角度を保持する数    //デックの大きさ

    /*  初期値0をデックに追加しておく   */
   	for(int i=0; i<NUM_JOINT; i++){
   		hold_data.shift_param[i].bias_hold.resize(max_hold_num, 0);
   		hold_data.shift_param[i].kappa_hold.resize(max_hold_num, 0);
   		hold_data.shift_param[i].kappa_zero_pitch_hold.resize(max_hold_num, 0);
   		hold_data.shift_param[i].kappa_zero_yaw_hold.resize(max_hold_num, 0);
   		hold_data.shift_param[i].tau_hold.resize(max_hold_num, 0);
   		hold_data.shift_param[i].psi_hold.resize(max_hold_num, 0);
   		hold_data.shift_param[i].psi_hyper_hold.resize(max_hold_num, 0);
   	}
}

/***
 *  汎用シフト制御  先頭から
 *  サーペノイド曲線の場合，曲率kappa, bias など
 *  常螺旋曲線の場合，シフトするパラメータは曲率kappa, 捩率tauとpsi
 * */
void ShiftControlMethod::Shift_Param_Forward(RobotSpec spec)
{
	int NUMOFLINK = spec.num_joint() ;

	//  ヘビ関節の vector をクリアする
	snake_model_param.bias.clear();
	snake_model_param.kappa.clear();
	snake_model_param.kappa_zero_pitch.clear();
	snake_model_param.kappa_zero_yaw.clear();
	snake_model_param.tau.clear();
	snake_model_param.psi.clear();
	snake_model_param.psi_hyper.clear();

	// kappa, tau, psi と psi_hyper を先頭ユニットの vector の先頭から入れる
	// 先頭デックの最初の要素に現在κ...を追加 (デックの長さ前より＋１になる)
	hold_data.shift_param[0].bias_hold.insert(hold_data.shift_param[0].bias_hold.begin(), bias_);
	hold_data.shift_param[0].kappa_hold.insert(hold_data.shift_param[0].kappa_hold.begin(), kappa_);
	hold_data.shift_param[0].kappa_zero_pitch_hold.insert(hold_data.shift_param[0].kappa_zero_pitch_hold.begin(), kappa_zero_pitch_);
	hold_data.shift_param[0].kappa_zero_yaw_hold.insert(hold_data.shift_param[0].kappa_zero_yaw_hold.begin(), kappa_zero_yaw_);
	hold_data.shift_param[0].tau_hold.insert(hold_data.shift_param[0].tau_hold.begin(), tau_);
	hold_data.shift_param[0].psi_hold.insert(hold_data.shift_param[0].psi_hold.begin(), psi_);
	hold_data.shift_param[0].psi_hyper_hold.insert(hold_data.shift_param[0].psi_hyper_hold.begin(), psi_hyper_);

	// 一つ前の関節の vector の最後のものを次の関節の vector の先頭から追加する
	int hold_num = (int)hold_data.shift_param[0].kappa_hold.size();   // --> max hold num + 1
	//ROS_INFO("hold_num size = %d", hold_num);
	for(int i=1; i<NUMOFLINK; i++){
		hold_data.shift_param[i].bias_hold.insert(hold_data.shift_param[i].bias_hold.begin(), hold_data.shift_param[i-1].bias_hold[hold_num-1]);
		hold_data.shift_param[i].kappa_hold.insert(hold_data.shift_param[i].kappa_hold.begin(), hold_data.shift_param[i-1].kappa_hold[hold_num-1]);
		hold_data.shift_param[i].kappa_zero_pitch_hold.insert(hold_data.shift_param[i].kappa_zero_pitch_hold.begin(), hold_data.shift_param[i-1].kappa_zero_pitch_hold[hold_num-1]);
		hold_data.shift_param[i].kappa_zero_yaw_hold.insert(hold_data.shift_param[i].kappa_zero_yaw_hold.begin(), hold_data.shift_param[i-1].kappa_zero_yaw_hold[hold_num-1]);
		hold_data.shift_param[i].tau_hold.insert(hold_data.shift_param[i].tau_hold.begin(), hold_data.shift_param[i-1].tau_hold[hold_num-1]);
		hold_data.shift_param[i].psi_hold.insert(hold_data.shift_param[i].psi_hold.begin(), hold_data.shift_param[i-1].psi_hold[hold_num-1]);
		hold_data.shift_param[i].psi_hyper_hold.insert(hold_data.shift_param[i].psi_hyper_hold.begin(), hold_data.shift_param[i-1].psi_hyper_hold[hold_num-1]);
	}

	// 角度保持 vector の末尾のκ... などのデータを読み取る   -->  ヘビ関節 vector の末尾から追加する
	// _________________________________________________________________
	// | 一番初めに入れたデータ | <-- | <-- | <-- | <-- | <-- | <-- |   <--    <--    <--  | <---     |
	// |________先頭________|___|___|___|___|___|___|_____________|_末尾_|
	//    0                  1   2   3   4   5   6    ........       n
	for(int i=0; i<NUMOFLINK; i++){

   	    snake_model_param.bias.push_back(hold_data.shift_param[i].bias_hold[hold_num-1]);
        snake_model_param.kappa.push_back(hold_data.shift_param[i].kappa_hold[hold_num-1]);
        snake_model_param.kappa_zero_pitch.push_back(hold_data.shift_param[i].kappa_zero_pitch_hold[hold_num-1]);
        snake_model_param.kappa_zero_yaw.push_back(hold_data.shift_param[i].kappa_zero_yaw_hold[hold_num-1]);
   	    snake_model_param.tau.push_back(hold_data.shift_param[i].tau_hold[hold_num-1]);
   	    snake_model_param.psi.push_back(hold_data.shift_param[i].psi_hold[hold_num-1]);
        snake_model_param.psi_hyper.push_back(hold_data.shift_param[i].psi_hyper_hold[hold_num-1]);

        // 角度保持 vectorは＋１になりましたので，vector 最後の要素を削除する     ----> max hold num - 1
		hold_data.shift_param[i].bias_hold.pop_back();
		hold_data.shift_param[i].kappa_hold.pop_back();
		hold_data.shift_param[i].kappa_zero_pitch_hold.pop_back();
		hold_data.shift_param[i].kappa_zero_yaw_hold.pop_back();
		hold_data.shift_param[i].tau_hold.pop_back();
		hold_data.shift_param[i].psi_hold.pop_back();
		hold_data.shift_param[i].psi_hyper_hold.pop_back();
	}
}

/***
 *  汎用シフト制御  末尾から
  *    サーペノイド曲線の場合，曲率kappa, bias など
 *  常螺旋曲線の場合，シフトするパラメータは曲率kappa, 捩率tauとpsi
 * */
void ShiftControlMethod::Shift_Param_Back(RobotSpec spec)
{
	int NUMOFLINK = spec.num_joint() ;

	//  ヘビ関節の vector をクリアする
	snake_model_param.bias.clear();
	snake_model_param.kappa.clear();
	snake_model_param.tau.clear();
	snake_model_param.psi.clear();
	snake_model_param.psi_hyper.clear();

	// kappa, tau, psi と psi_hyper を先頭ユニットの vector の先頭から入れる
	// 先頭デックの最初の要素に現在κ...を追加 (デックの長さ前より＋１になる)
	hold_data.shift_param[0].bias_hold.insert(hold_data.shift_param[0].bias_hold.begin(), bias_);
	hold_data.shift_param[0].kappa_hold.insert(hold_data.shift_param[0].kappa_hold.begin(), kappa_);
	hold_data.shift_param[0].kappa_zero_pitch_hold.insert(hold_data.shift_param[0].kappa_zero_pitch_hold.begin(), kappa_zero_pitch_);
	hold_data.shift_param[0].kappa_zero_yaw_hold.insert(hold_data.shift_param[0].kappa_zero_yaw_hold.begin(), kappa_zero_yaw_);
	hold_data.shift_param[0].tau_hold.insert(hold_data.shift_param[0].tau_hold.begin(), tau_);
	hold_data.shift_param[0].psi_hold.insert(hold_data.shift_param[0].psi_hold.begin(), psi_);
	hold_data.shift_param[0].psi_hyper_hold.insert(hold_data.shift_param[0].psi_hyper_hold.begin(), psi_hyper_);

	// 一つ前の関節の vector の最後のものを次の関節の vector の先頭から追加する
	int hold_num = (int)hold_data.shift_param[0].kappa_hold.size();   // --> max hold num + 1
	//ROS_INFO("hold_num size = %d", hold_num);
	for(int i=1; i<NUMOFLINK; i++){
		hold_data.shift_param[i].bias_hold.insert(hold_data.shift_param[i].bias_hold.begin(), hold_data.shift_param[i-1].bias_hold[hold_num-1]);
		hold_data.shift_param[i].kappa_hold.insert(hold_data.shift_param[i].kappa_hold.begin(), hold_data.shift_param[i-1].kappa_hold[hold_num-1]);
		hold_data.shift_param[i].kappa_zero_pitch_hold.insert(hold_data.shift_param[i].kappa_zero_pitch_hold.begin(), hold_data.shift_param[i-1].kappa_zero_pitch_hold[hold_num-1]);

		hold_data.shift_param[i].kappa_zero_yaw_hold.insert(hold_data.shift_param[i].kappa_zero_yaw_hold.begin(), hold_data.shift_param[i-1].kappa_zero_yaw_hold[hold_num-1]);
		hold_data.shift_param[i].tau_hold.insert(hold_data.shift_param[i].tau_hold.begin(), hold_data.shift_param[i-1].tau_hold[hold_num-1]);
		hold_data.shift_param[i].psi_hold.insert(hold_data.shift_param[i].psi_hold.begin(), hold_data.shift_param[i-1].psi_hold[hold_num-1]);
		hold_data.shift_param[i].psi_hyper_hold.insert(hold_data.shift_param[i].psi_hyper_hold.begin(), hold_data.shift_param[i-1].psi_hyper_hold[hold_num-1]);
	}

	// 角度保持 vector の末尾のκ... などのデータを読み取る   -->  ヘビ関節 vector の先頭から追加する
	// _________________________________________________________________
	// | 一番初めに入れたデータ | <-- | <-- | <-- | <-- | <-- | <-- |   <--    <--    <--  | <---     |
	// |_______末尾_________|___|___|___|___|___|___|_____________|_先頭_|
	//    n                  6   5   4   3   2   1    ........       0
	for(int i=0; i<NUMOFLINK; i++){
		snake_model_param.kappa.insert(snake_model_param.kappa.begin(), hold_data.shift_param[i].kappa_hold[hold_num-1]);
		snake_model_param.kappa_zero_pitch.insert(snake_model_param.kappa_zero_pitch.begin(), hold_data.shift_param[i].kappa_zero_pitch_hold[hold_num-1]);
		snake_model_param.kappa_zero_yaw.insert(snake_model_param.kappa_zero_yaw.begin(), hold_data.shift_param[i].kappa_zero_yaw_hold[hold_num-1]);
   	snake_model_param.tau.insert(snake_model_param.tau.begin(), hold_data.shift_param[i].tau_hold[hold_num-1]);
   	snake_model_param.bias.insert(snake_model_param.bias.begin(), hold_data.shift_param[i].bias_hold[hold_num-1]);
   	snake_model_param.psi.insert(snake_model_param.psi.begin(), hold_data.shift_param[i].psi_hold[hold_num-1]);
  	snake_model_param.psi_hyper.insert(snake_model_param.psi_hyper.begin(), hold_data.shift_param[i].psi_hyper_hold[hold_num-1]);

        // 角度保持 vectorは＋１になりましたので，vector 最後の要素を削除する     ----> max hold num - 1
		hold_data.shift_param[i].bias_hold.pop_back();
		hold_data.shift_param[i].kappa_hold.pop_back();
		hold_data.shift_param[i].kappa_zero_pitch_hold.pop_back();
		hold_data.shift_param[i].kappa_zero_yaw_hold.pop_back();
		hold_data.shift_param[i].tau_hold.pop_back();
		hold_data.shift_param[i].psi_hold.pop_back();
		hold_data.shift_param[i].psi_hyper_hold.pop_back();
	}
}

/***
 *  tau のシフト制御
 *   this method is NOT USED in heliacal wave propagate motion
 * */
void ShiftControlMethod::ShiftParamTorsion(RobotSpec spec)
{
	snake_model_param.tau.clear();

	int NUMOFLINK = spec.num_joint() ;
	/***  tauを先頭ユニットのDEQUEの先頭に入れる  ***/
	/*** 先頭デックの最初の要素に現在tauを追加 (デックの長さ前より＋１になる)  ***/
	hold_data.shift_param[0].tau_hold.insert(hold_data.shift_param[0].tau_hold.begin(), tau_);
		/***  一つ前の関節のデックの最後のものを次の関節のDEQUEの先頭に追加する  ***/
	int hold_num = (int)hold_data.shift_param[0].tau_hold.size();
	for(int i=1; i<NUMOFLINK; i++){
		hold_data.shift_param[i].tau_hold.insert(hold_data.shift_param[i].tau_hold.begin(),
        		  hold_data.shift_param[i-1].tau_hold[hold_num-1]);
	}
	/*  角度保持デックの末尾のtauを読み取る, ヘビの関節に送る  */
	for(int i=0; i<NUMOFLINK; i++){
		snake_model_param.tau.push_back(hold_data.shift_param[i].tau_hold[hold_num-1]);
		hold_data.shift_param[i].tau_hold.pop_back();
	}
}

/***
 *   psi のシフト制御
 *   this method is NOT USED in heliacal wave propagate motion
 * */
void ShiftControlMethod::ShiftParamPsi(RobotSpec spec)
{
	snake_model_param.psi.clear();

	int NUMOFLINK = spec.num_joint() ;
	/***  kappaとtauを先頭ユニットのDEQUEの先頭に入れる  ***/
	/*** 先頭デックの最初の要素に現在psiを追加 (デックの長さ前より＋１になる)  ***/
	hold_data.shift_param[0].psi_hold.insert(hold_data.shift_param[0].psi_hold.begin(), psi_);
	/***  一つ前の関節のデックの最後のものを次の関節のDEQUEの先頭に追加する  ***/
	int hold_num = (int)hold_data.shift_param[0].kappa_hold.size();
	for(int i=1; i<NUMOFLINK; i++){
		hold_data.shift_param[i].psi_hold.insert(hold_data.shift_param[i].psi_hold.begin(), hold_data.shift_param[i-1].psi_hold[hold_num-1]);
	}
	/*  角度保持デックの末尾のpsiを読み取る, ヘビの関節に送る  */
	for(int i=0; i<NUMOFLINK; i++){
		snake_model_param.psi.push_back(hold_data.shift_param[i].psi_hold[hold_num-1]);
		hold_data.shift_param[i].psi_hold.pop_back();
	}
}

/***
 *   Helical Wave Propagate Motion のための曲率 kappa のシフト制御
 *   螺旋進行波はヘビのしっぽからおくるため
 * */
void ShiftControlMethod::ShiftParamCurvatureForHelicalWave(RobotSpec spec)
{
	snake_model_param.kappa.clear();

	int NUMOFLINK = spec.num_joint() ;
	hold_data.shift_param[0].kappa_hold.insert(hold_data.shift_param[0].kappa_hold.begin(), kappa_);

	//  一つ前の関節の VECTOR の最後のものを次の関節の VECTOR の先頭に追加する
	int hold_num = (int)hold_data.shift_param[0].kappa_hold.size();

	for(int i=1; i<NUMOFLINK; i++){
		hold_data.shift_param[i].kappa_hold.insert(hold_data.shift_param[i].kappa_hold.begin(),
				hold_data.shift_param[i-1].kappa_hold[hold_num-1]);
	}

	// 角度保持 vector の末尾の kappa のデータを読み取る   -->  ヘビ関節 vector の先頭から追加する
	//螺旋進行波はヘビのしっぽからおくるため
	// _____________________________________________________________________________
	// | --> | --> | --> | --> | --> | --> | --> | --> --> --> | 一番初めに入れたデータ |
	// |_先頭_|_____|_____|_____|_____|_____|_____|_____________|_______末尾_________|
	//    0     1     2     3     4     5     6    ............          n
	for(int i=0; i<NUMOFLINK; i++){
		snake_model_param.kappa.insert(snake_model_param.kappa.begin(), hold_data.shift_param[i].kappa_hold[hold_num-1]);
		// 角度保持 vectorは＋１になりましたので，vector 最後の要素を削除する
		hold_data.shift_param[i].kappa_hold.pop_back();
	}
}

/***
 *   Helical Wave Propagate Motion のための   psi_hyper のシフト制御
 *	 螺旋進行波はヘビのしっぽからおくるため
 * */
void ShiftControlMethod::ShiftParamPsiHyperForHelicalWave(RobotSpec spec)
{
	snake_model_param.psi_hyper.clear();
	int NUMOFLINK = spec.num_joint() ;

	//  psi_hyper を先頭ユニットのDEQUEの先頭から入れる
	//  先頭デックの最初の要素に現在 psi_hyper を追加 (デックの長さ前より＋１になる)
	hold_data.shift_param[0].psi_hyper_hold.insert(hold_data.shift_param[0].psi_hyper_hold.begin(), psi_hyper_);

	//  一つ前の関節の VECTOR の最後のものを次の関節の VECTOR の先頭から追加する  ***/
	int hold_num = (int)hold_data.shift_param[0].psi_hyper_hold.size();
	for(int i=1; i<NUMOFLINK; i++){
		hold_data.shift_param[i].psi_hyper_hold.insert(hold_data.shift_param[i].psi_hyper_hold.begin(), hold_data.shift_param[i-1].psi_hyper_hold[hold_num-1]);
	}

	// 角度保持 VECTOR の末尾の psi_hyper を読み取る --> ヘビ関節 vector の先頭から追加する
	//  螺旋進行波はヘビのしっぽからおくるため
	// _____________________________________________________________________________
	// | --> | --> | --> | --> | --> | --> | --> | --> --> --> | 一番初めに入れたデータ |
	// |_先頭_|_____|_____|_____|_____|_____|_____|_____________|_______末尾_________|
	//    0     1     2     3     4     5     6    ............          n
	for(int i=0; i<NUMOFLINK; i++){
		snake_model_param.psi_hyper.insert(snake_model_param.psi_hyper.begin(), hold_data.shift_param[i].psi_hyper_hold[hold_num-1]);
		hold_data.shift_param[i].psi_hyper_hold.pop_back();
	}
}

/***
 *   20190126 IMUN  for CCC2019
 *   Helical Wave Propagate Motion のための   psi_hyper のシフト制御
 *	 螺旋進行波はヘビのしっぽからおくるため
 * */
void ShiftControlMethod::ShiftParamPsiHyperForHelicalWaveForward(RobotSpec spec)
{
	snake_model_param.psi_hyper.clear();
	int NUMOFLINK = spec.num_joint() ;

	//  psi_hyper を先頭ユニットのDEQUEの先頭から入れる
	//  先頭デックの最初の要素に現在 psi_hyper を追加 (デックの長さ前より＋１になる)
	hold_data.shift_param[0].psi_hyper_hold.insert(hold_data.shift_param[0].psi_hyper_hold.begin(), psi_hyper_);
//hold_data.shift_param[0].psi_hyper_hold.insert(hold_data.shift_param[0].psi_hyper_hold.begin(), psi_hyper_);

	//  一つ前の関節の VECTOR の最後のものを次の関節の VECTOR の先頭から追加する  ***/
	int hold_num = (int)hold_data.shift_param[0].psi_hyper_hold.size();
	for(int i=1; i<NUMOFLINK; i++){
		hold_data.shift_param[i].psi_hyper_hold.insert(hold_data.shift_param[i].psi_hyper_hold.begin(), hold_data.shift_param[i-1].psi_hyper_hold[hold_num-1]);
	}

	// 角度保持 VECTOR の末尾の psi_hyper を読み取る --> ヘビ関節 vector の先頭から追加する
	//  螺旋進行波はヘビのしっぽからおくるため
	// _____________________________________________________________________________
	// | --> | --> | --> | --> | --> | --> | --> | --> --> --> | 一番初めに入れたデータ |
	// |_先頭_|_____|_____|_____|_____|_____|_____|_____________|_______末尾_________|
	//    0     1     2     3     4     5     6    ............          n
	for(int i=0; i<NUMOFLINK; i++){
		snake_model_param.psi_hyper.push_back(hold_data.shift_param[i].psi_hyper_hold[hold_num-1]);
		//snake_model_param.psi_hyper.insert(snake_model_param.psi_hyper.begin(), hold_data.shift_param[i].psi_hyper_hold[hold_num-1]);
		hold_data.shift_param[i].psi_hyper_hold.pop_back();
	}
}


/***
 *   Helical Wave Propagate Motion のための   psi_hyper のシフト制御
 *	 螺旋進行波はヘビのしっぽからおくるため
 * */
void ShiftControlMethod::ShiftParamPsiForHelicalWave(RobotSpec spec)
{
	snake_model_param.psi.clear();
	int NUMOFLINK = spec.num_joint() ;

	//  psi_hyper を先頭ユニットのDEQUEの先頭から入れる
	//  先頭デックの最初の要素に現在 psi_hyper を追加 (デックの長さ前より＋１になる)
	hold_data.shift_param[0].psi_hold.insert(hold_data.shift_param[0].psi_hold.begin(), psi_hyper_);

	//  一つ前の関節の VECTOR の最後のものを次の関節の VECTOR の先頭から追加する  ***/
	int hold_num = (int)hold_data.shift_param[0].psi_hold.size();
	for(int i=1; i<NUMOFLINK; i++){
		hold_data.shift_param[i].psi_hold.insert(hold_data.shift_param[i].psi_hold.begin(), hold_data.shift_param[i-1].psi_hold[hold_num-1]);
	}

	// 角度保持 VECTOR の末尾の psi_hyper を読み取る --> ヘビ関節 vector の先頭から追加する
	//  螺旋進行波はヘビのしっぽからおくるため
	// _____________________________________________________________________________
	// | --> | --> | --> | --> | --> | --> | --> | --> --> --> | 一番初めに入れたデータ |
	// |_先頭_|_____|_____|_____|_____|_____|_____|_____________|_______末尾_________|
	//    0     1     2     3     4     5     6    ............          n
	for(int i=0; i<NUMOFLINK; i++){
		snake_model_param.psi.insert(snake_model_param.psi.begin(), hold_data.shift_param[i].psi_hold[hold_num-1]);
		hold_data.shift_param[i].psi_hold.pop_back();
	}
}
