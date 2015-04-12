■サンプル名：
read_parameters_from_sdc

■サンプル概要：
実行時に使用されるパラメータをSDカードから読み込みます.

■サンプル説明：
Micor SDカードに書き込まれた、param.txt を読み込んで実行時の挙動を変化させます。max_duty = XXX(0～100) という形で設定されたパラメータを読み込み、設定された
DUTY比でモーターを駆動します。

■実行方法
1．プログラムを制御基板に書き込む
2. param.txt をMicro SDカードにコピーする
3. 1で用意したMicro SDカードを制御基板のMicro SDカードスロットに挿入する
4．電源を入れる

■注意点：


■ETC.
Witten by Akasaka