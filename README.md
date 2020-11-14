# abu2021

## 必要パッケージ
sudo apt install python-catkin-tools  
sudo pip3 install rospkg catkin_pkg  

## パッケージ説明
abu2021_msgs：メッセージ用パッケージ

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
