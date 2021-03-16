# abu2021

## 必要パッケージ
sudo apt install python-catkin-tools  
sudo pip3 install rospkg catkin_pkg  
sudo apt install ros-melodic-joy  
sudo apt install ros-melodic-joystick-drivers  
sudo apt install ros-melodic-rosserial-arduino  
sudo apt install ros-melodic-rosserial   

## 注意
$ git clone 後$ catkin build できない場合，$ catkin build trkikou と$ catkin build abu2021_msgs を先にやってから$ catkin build するといける．

## パッケージ・ノード説明

### パッケージ名
パッケージの説明
```
実行形式ファイル名
　ノードの説明
  param  
  変数の型　変数名：　説明  
  pub,sub  
  [トピック名]（メッセージ型）：説明  
```
### task_manager
ジョイコンからの司令を各機構へ司令に変換する
```
task_manager_dr
　DRのタスクマネージャ
  sub  
  [joy] (sensor_msgs::Joy)：コントローラの入力．左スティックでxy速度，右スティックでyaw角速度  
  pub
  [cmd] (abu2021_msgs::cmd_vw)：足回りモデルへ送る目標速度・角速度（正面x，鉛直zの右手系） 
  [ad_order] (abu2021_msgs::auto_drive_order)：自動走行の指令．緊急停止，目標経路の決定，自己位置の再設定等を行う（ボタン未確定）
```
```
task_manager_tr
　TRのタスクマネージャ
  sub  
  [joy] (sensor_msgs::Joy)：コントローラの入力．左スティックでxy速度，右スティックでyaw角速度  
  pub
  [cmd] (abu2021_msgs::cmd_vw)：足回りモデルへ送る目標速度・角速度（正面x，鉛直zの右手系）  
  [tr_order] (std_msgs::Int32)：TRへ送る動作指示. □ボタン又は×ボタンで投擲動作の緊急停止，△ボタンで装填，○ボタンで射出．
```
### kinematics_model
メカナム・オムニの動力学モデルを利用して各モータの指令値を計算
```
kinematics_model
  sub
  [cmd] (abu2021_msgs::cmmd_vw): 説明略
  pub
  [motor_vel] (abu2021_msgs::motor_pw): 各モーターの速度指令
※補足: cmd_trをサブスクライブする場合、パッケージ内のconstant_tr.yamlをパラメータサーバーに送ってノードを起動する必要がある
同様に、cmd_drをサブスクライブする場合、constant_dr.yamlをパラメータサーバーに送ってノードを起動する必要がある
```
### trkikou
TRのタスクマネージャからの司令をArduinoへの司令に変換する
```
new_touteki_talker
  sub
  [touteki_enc](std_msgs::Int64)：エンコーダから得た角度をArduinoから受け取る．
  [tr_order](std_msgs::Int32)：TRのタスクマネージャからの司令を受け取る.
  pub
  [touteki_sizi](trkikou::sizi)：Arduinoへの司令．mode(0:pwを読む,1:PID)，deg(mode=1での目標角度)，pw(mode=0でのモータ出力)，solenoid(0:緩める,1:開ける,2:閉める)
  [touteki_node_debug](std_msgs::Int64MultiArray)：デバッグ用. [step_pick,step_launch,count_pick]
```
### auto_drive
自動走行用パッケージ．経路生成プログラム等含む．
```
auto_driving
  自動走行の目標経路から，目標速度・角速度を出す（pure pursuit）
  sub
  [ad_order] (abu2021_msgs::auto_drive_order)：ジョイコンからの自動走行の指令．緊急停止，目標経路の決定，自己位置の再設定等を行う（ボタン未確定）
  [gyro_yaw] (std_msgs::Float64)：カルマンフィルタをかけた後のジャイロセンサの値[rad]
  [odometer] (std_msgs::odom_rad)：オドメータの各車輪の変化角度[rad,rad]
  pub
  [cmd](trkikou::sizi)：足回りモデルへ送る目標速度・角速度[m/s,m/s,rad/s]
```

### signal_processiong
自己位置推定等の信号処理を行う
```
gyro_kalman
  ジャイロセンサの生値と目標角速度から，カルマンフィルタをかけた機体角度を出力
  sub
  [gyro_raw] (std_msgs::Float64)：IMUセンサからの角速度の生値[deg/s]
  [odometer] (std_msgs::odom_rad)：オドメータの各車輪の変化角度[rad,rad]
  pub
  [gyro_yaw](std_msgs::Float64)：カルマンフィルタをかけた後のジャイロセンサの値[rad]
```
```
dead_reckoning
  ジャイロセンサの生値とオドメータの値から相対座標を出力
  sub
  [gyro_raw] (std_msgs::Float64)：IMUセンサからの角速度の生値[deg/s]
  [odometer] (abu2021_msgs::odom_rad)：機体の目標角速度[rad/s]
  tf broadcast
  base_link->odom：機体の相対座標[m,m,rad]
```
### abu2021_msgs
メッセージ用パッケージ．使うカスタムメッセージは全部ここに入れる．ノード無し

## UTM-30LX-EWの使用方法
1. URGの電源を入れる. 茶色のケーブルを12V、青色のケーブルをGNDに接続する.
2. URGから伸びているLANケーブルをPCと接続する.
3. ターミナルにifconfigと入力し、イーサネットのインターフェース名を調べ、メモする.
4. (有線接続できませんでしたなどとメッセージが表示された後に)sudo ifconfig (イーサネットのインターフェース名) (ipアドレス)とターミナルに入力し、PCのipアドレスを変更する.  
   **注意: ipアドレスには、URGに設定されているipアドレスをもとに適当なものを指定する. 例えば、URGのipアドレスが192.168.0.10なら、192.168.0.xxx(xxxは10以外の値)を指定する. ちなみに、デフォルトで設定されているURGのipアドレスは192.168.0.10である. また、172.16.0.10に設定したものもあるので注意.**  
5. rosrun urg_node urg_node _ip_address:=(URGのipアドレス)をターミナルに打って実行し、streaming dataと表示されれば正常に接続されている.

## システム関係図
![system_diagram.png](https://github.com/tsukurobo/abu2021/blob/main/README/system_diagram.png)

## モデル
![omni_model.jpg](https://github.com/tsukurobo/abu2021/blob/main/README/omni_model.jpg)

## 仕様
- catkin buildを標準に  
https://qiita.com/harumo11/items/ae604ba2e17ffda529c2  
- python3を標準に  
 $sudo pip3 install rospkg catkin_pkg  
 pythonスクリプトの先頭を「#!/usr/bin/env python3」に変えるとpython3で実行されるようになる  
- メッセージはメッセージ専用パッケージを使う  
- パッケージ名は大文字無し  
