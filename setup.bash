# rosのアレを有効か
source opt/ros/noetic/setup.bash

# ROS_MASTER_URIを設定
export ROS_MASTER_URI=http://hsrc.local:11311/

# venvフォルダが存在しない場合は作成
if [ ! -d "venv" ]; then
    python3 -m venv venv
fi

# venvフォルダ内のactivateを実行
source venv/bin/activate

# 必要なライブラリをインストール
pip install -r requirements.txt