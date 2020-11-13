# abu2021

## パッケージ
sudo apt install python-catkin-tools  
sudo pip3 install rospkg catkin_pkg  

# モデル
![omni_model.jpg](https://github.com/tsukurobo/abu2021/blob/main/README/omni_model.jpg)

## 仕様
- catkin buildを標準に  
https://qiita.com/harumo11/items/ae604ba2e17ffda529c2  
- python3を標準に  
 $sudo pip3 install rospkg catkin_pkg  
 pythonスクリプトの先頭を「#!/usr/bin/env python3」に変えるとpython3で実行されるようになる  
- メッセージはメッセージ専用パッケージを使う  
- パッケージ名は大文字無し  