# ODE

* Open Dynamics Engineをつかったシミュレーションプログラムたち
* branchは「master」、CmakeLists.txtがUbuntuとMacと微妙に違うのでそこでブランチを分けている

## [1. Nachi](https://github.com/Ry0/ODE/tree/master/Nachi)
株式会社不二越のNachi、MZ07のシミュレーション  
![Nachi](https://dl.dropboxusercontent.com/u/23873125/Markdown/Nachi_MZ072.jpg)

### 更新情報
* 2014.12.16 ファイル引数を指定するようにしたよ
* 2014.12.19 RRTの枝を表示するようにした。
* ただし関数は冗長  

## [2. RobotArmTest](https://github.com/Ry0/ODE/tree/master/RobotArmTest)
ロボットアームのシミュレーションの準備

### inverseKinematics
* 逆運動学の解で、解が存在しなかったらどうするのかをエラー処理したコード

### arm
* めっちゃ初期の段階のロボットアームのソースコード
* 3軸と6軸が混ざってる  
![arm](https://dl.dropboxusercontent.com/u/23873125/Markdown/arm.jpg)

### etc
* 他のODEで作ったシミュレーションと出村先生のサンプルコード  




