# abu2021

## 必要パッケージ
sudo apt-get install python-catkin-tools  
sudo pip3 install rospkg catkin_pkg  
sudo apt-get install ros-melodic-joy
sudo apt-get install ros-melodic-joystick-drivers

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
  [cmd_dr] (abu2021_msgs::cmd_vw)：足回りモデルへ送る目標速度・角速度
```
### auto_drive_sim
自動走行用パッケージ．経路生成プログラム等含む．まだノード無し．

### abu2021_msgs
メッセージ用パッケージ．使うカスタムメッセージは全部ここに入れる．ノード無し


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
