hand-eye-quat3.wolframの実データ版でcx,cy(cu,cv)の入れ方が間違っている可能性あり。要確認


変更予定点は2点

1. poses(pose1,pose2,pose3,pose4,pose5,pose6)は実機(rosbag)では
   Vector3+Quaternionなので、poseTの定義を(rotx,y,zからquat2matに)変更する
2. EulerAngleからQuaternion虚部3パラメーターに変更して
   収束精度があがるかどうかを見る

