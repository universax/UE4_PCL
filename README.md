# UE4_PCL
PCL Plugin Example for UE4

UE4.21で動作確認済

## Requirement
VLP16のGrabberなどもパッケージしたので、Packet capture系をインストールしておく必要があります。（下記など）
http://www.win10pcap.org/ja/

## PCL一式
Plugins\PCLPlugin\Source\ThirdPartyに各種ライブラリが入っています。
それぞれStaticにビルドしなおしたもので、こうしないとプラグイン化できなかったので、
ignoreせずに全部入ってます。
なので、pullすごい時間かかると思います、、、

## Exampleに関して
PCDを読み込む感じになってます。
Plugins\PCLPlugin\Content以下にteapot.pcdいれておきました。
PCL_Actor内で絶対パスを適当に食わせて読み込んでるので、各々の環境に合わせて書き換えてください。

