# wili_hmm
更新アルゴリズムに不備アリ。
0除算が発生する。

## WiLIについて
なくしもの捜索用ROS2パッケージ群です。<br>
名前はWhere is a Lost Itemの頭文字から取りました。

## 概要
以下の機能を含むROS2パッケージです。<br>

* 利用者の位置推移を用いてHMM(隠れマルコフモデル)の遷移確率など各種パラメータを更新

言語はC++です。

## 依存
* [wili_msgs](https://github.com/MaruKazeMaru/wili_msgs)<br>
WiLIで用いるメッセージ、サービスを定義しています。

## トピック
### /observation
wili_msgs/msg/Observation 型<br>
利用者の位置推移を表します。

### /new_hmm
wili_msgs/msg/HMM 型<br>
更新後のHMMのパラメータを表します。

## サービス
### /get_hmm
wili_msgs/srv/GetHMM 型<br>
HMMのパラメータ取得に用います。

## ノード
### baum_welch
/observationトピックをsubscribeするとバウム・ウェルチアルゴリズムによりHMMのパラメータを更新し、更新後のパラメータを/new_hmmトピックとしてpublishします。<br>
HMMのパラメータの初期化には/get_hmmサービスを用います。

## ライセンス
MITライセンスです。<br>
[LICENSE](./LICENSE)をお読みください。

## 参考
1. taka256．”pythonでHMMのパラメータ推定実装”．Qiita．2016．[https://qiita.com/taka256/items/3e5306d0432c05909992](https://qiita.com/taka256/items/3e5306d0432c05909992)，（2023）
1. 上田修功，石井健一郎．続・わかりやすい　パターン認識．オーム社，2014．
1. 中川聖一，平田好充，橋本泰秀．連続出力分布型HMMによる日本語音韻認識．日本音響学会誌．1990，46巻，6号．p.486-496．
