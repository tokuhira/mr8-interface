# mr8-interface

KO PROPO製のプロポセットMC-8/MR-8にマルチプレクサ機能等を追加するプロジェクトです。

# Features

CMOSロジックICのマルチプレクサを利用してプロポからの操作と別系統からの操作を選択できます。
操作系統の切り替えはMC-8の操作スティック押下で行います。
CH-Eを押すと別系統の操作が有効になり、CH-Fを押すとプロポによる手動操作に戻ります。
別系統との電気信号は絶縁されているため、異なる電源系統のユニットとも接続できます。
おまけ機能で両系統のステアリングともにリバース信号も出力します。
おまけ機能でプロポ操作に対応するS.BUS信号も出力する予定ですが、プログラムは未実装です。

# Requirement

Adafruit QT Pyが別途必要です。

# Installation

このボードとAdafruit QT Pyは直接ハンダ付けします。
このボードとMR-8の接続については回路図を参照のうえ、ハーネスもしくは添付のミニ基板を使用してください。
このボードとRCシステム（スロットルに対応するESC、ならびにステアリングに対応するサーボ）の接続についても回路図を参照してください。
TAKACHI製のプラスチックケースTW4-2-5に格納することができます。
TAKACHI製のプラスチックケースSW-40にも格納することができますが、その場合はマウントホールのはみ出し部を切り落とす必要があります。

# Usage

各々方でご自由に応用していただけます。

# Note

可用性、安全性、その他いかなる性質も保証しません。
いかなる用法も、またのその結果も保証しません。

# Author

* @tokuhira (twitter)
* github.com/tokuhira

# License

BSD 3-Clause License

Copyright (c) 2020, tokuhira.net
All rights reserved.
