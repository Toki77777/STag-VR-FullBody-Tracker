# STag-VR-FullBody-Tracker

STag マーカーを使った VR 用フルボディトラッキング。

これは、fiducial マーカー（位置検出用マーカー）を使ってフルボディトラッキングシステムを作る試みです。これにより、スマホと段ボールだけで無料でフルボディトラッキングを実現できます。マーカーのサイズが 10cm 程度と小さく、640x480 解像度の PS Eye カメラでも、かなり良好なトラッキングが可能です。マーカーを大きくしたり、より高解像度で高速なスマホカメラを使ったりすれば、トラッキング精度はさらに向上します。

**注意: これは無料かつオープンソースのプロジェクトです。このために DRIVER4VR は必要ありません！**

使用するには、トラッカーを 3 つ作る必要があります — 両脚に 1 つずつ、腰に 1 つです。脚のトラッカーだけでは VRChat では動作しません！

このバージョンは、安定していてオクルージョン（遮蔽）に強い fiducial マーカーシステムである STag を採用しており、GUI やより分かりやすいキャリブレーションなど、システムを使いやすくするための多くの改良が加えられています。

### マーカーの印刷

[images-to-print](images-to-print) フォルダには、印刷してすぐに使える A4 の STag（HD11）マーカーシートが入っています。マーカーは 93mm（黒い正方形の辺の長さ）で、デフォルトのトラッカー構成に合わせてサイズと配置が調整されています。各ページには 1 つのトラッカーの表面（front）または裏面（back）のマーカーペアが配置されているため、トラッカーごとに表面ページ 1 枚と裏面ページ 1 枚を印刷してください（同じトラッカーの表面シートと裏面シートには、その同一トラッカーに属する異なるマーカー ID が印刷されています）。より多くのマーカーが必要な場合や、別サイズのマーカーが必要な場合は、[utilities/generate_stag_markers.py](utilities/generate_stag_markers.py) で自分で生成できます（numpy、opencv-python、pillow をインストールした Python が必要です）。

プログラムは [releases](https://github.com/Toki77777/STag-VR-FullBody-Tracker/releases) タブからダウンロードできます。

![demo](images/demo.gif)

### 簡単なセットアップ動画:
近日公開

## HMD 参照マーカーによる自動キャリブレーション

カメラの位置・角度・スケールを手入力する代わりに、HMD に取り付けたマーカーボード（参照マーカー）から
カメラ姿勢を自動的に求められます。SteamVR が報告する HMD 姿勢と、カメラが写した参照マーカーの姿勢を
同じフレームで突き合わせると、カメラがどこにあるかが一意に決まります。Vive トラッカーのように、
置いて動かすだけで使えるようにするための機能です。

既定では無効です。有効化しなければ従来どおりの手動キャリブレーションのままです。

### 必要なもの

- SteamVR でトラッキングされている HMD（この機能は HMD 姿勢が取れないと何もしません）
- HMD に固定できるマーカーボード。マーカーは 2 枚以上必要です
- 既存のトラッカーが使っていないマーカー ID

### セットアップ手順

1. トラッカーが使っていない ID のマーカーを印刷します。トラッカーの ID 範囲は既定で
   1 トラッカーあたり `markersPerTracker`（既定 45）枚ずつなので、3 トラッカーなら 0〜134 を使います。
   重ならない範囲、たとえば 200〜209 を選んでください。マーカーは
   [utilities/generate_stag_markers.py](utilities/generate_stag_markers.py) で生成できます。
2. 印刷したマーカーを HMD に固定します。**HMD に対して動かないこと**が条件です。ずれると、
   ずれた分がそのままトラッキング全体のずれになります。
3. `config.yaml` の `referenceMarker` を設定します。ID 範囲は `[markerIdBegin, markerIdEnd)`
   の半開区間で、有効化するときは必ず明示指定が必要です（暗黙の既定範囲は割り当てられません）。
   ```yaml
   referenceMarker:
      enabled: 1
      markerIdBegin: 200
      markerIdEnd: 210
   ```
   範囲がトラッカー側と重なっている、または不正な場合は、参照マーカーだけが無効になり、
   トラッカーの ID 割り当ては一切変わりません。ログにエラーが出ます。
4. 通常のトラッカーキャリブレーションを実行します。参照マーカーは最後の対象として同じ手順で
   キャリブレーションされるので、トラッカーと同様にボードを見せてください。
5. トラッキングを開始し、カメラに参照マーカーを見せたまま、頭をゆっくり動かします。
   **1 軸だけ回すのでは足りません。**首を横に振る、うなずく、傾ける、と複数の軸で回してください。
   HMD とボードの位置関係が求まると、その値は `calib.yaml` に保存され、
   次回以降は参照マーカーが 1 フレーム写った時点でキャリブレーションが完了します。

### 動作と設定

- ボードが写っている間はカメラ姿勢を求め続けるので、三脚を蹴ってカメラが動いても自動的に復帰します。
  1 回の検出ミスでプレイスペースが飛ばないよう、大きく違う解は繰り返し出るまで採用しません。
- ボードが写っていないフレームは何もしません。直前のキャリブレーションがそのまま使われます。
- 手動キャリブレーションや multicam autocalib を開いている間は自動側が譲ります。閉じると自動側に戻ります。
- 求まった値は手動キャリブレーションの欄に反映されるので、数値としても確認できます。

| 設定 | 既定 | 内容 |
| --- | --- | --- |
| `referenceMarker.enabled` | `0` | 参照マーカーを使う |
| `referenceMarker.markerIdBegin` / `markerIdEnd` | `-1` | 参照マーカーの ID 範囲 `[begin, end)`。有効化時は必須 |
| `referenceMarker.continuousCalibration` | `1` | ボードが写っている間カメラ姿勢を追い続ける |
| `referenceMarker.recalibrateHmdOffset` | `0` | 保存済みの HMD とボードの位置関係を捨てて測り直す。**ボードを付け直したら 1 にしてください** |

### うまくいかないとき

プレビュー画面には、検出した参照マーカーの座標軸と、HMD 姿勢から予測したボード位置（黄色い点）の
両方が描かれます。この 2 つがずれている量が、そのままキャリブレーションのずれです。

`application.log` には、求まったカメラ姿勢（位置・角度・スケール）と、
参照マーカーから予測した HMD 姿勢と SteamVR が報告した HMD 姿勢の差が定期的に記録されます。
差が一定方向にずれ続けている場合はキャリブレーションが合っていません。
差がボードの動きに合わせてばらつく場合は検出精度が限界です。

キャリブレーションが完了しない場合は次を確認してください。

- 頭の回転が 1 軸に偏っていないか（この場合は原理的に解けないため、意図的に拒否します）
- ボードが HMD に対して動いていないか
- カメラキャリブレーションが妥当か。求まったスケールが 80〜120% の範囲を外れる場合は採用しません

## ビルド手順:

**注意: これは開発者向けです。~~コードを書くつもりはなく、アプリを使いたいだけの場合は、[Wiki](https://github.com/Toki77777/STag-VR-FullBody-Tracker/wiki) にチュートリアルがあります。~~**

本プロジェクトは CMake プロジェクトです。CMake とお好みの IDE / コンパイラを使ってビルドできますし、CMake プロジェクトを直接開ける IDE もあります。

### Linux での前提パッケージ
```
sudo apt-get update -y
sudo apt-get install -y build-essential tar curl zip unzip pkg-config autoconf libudev-dev freeglut3-dev libgtk-3-dev libsecret-1-dev libgcrypt20-dev libsystemd-dev ffmpeg
```
OpenCV の gstreamer バックエンド
```
sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
```


### Windows での前提
Visual Studio で開くか、Visual Studio コマンドプロンプトを使用してください。


### クローンとビルド
```
git clone https://github.com/Toki77777/STag-VR-FullBody-Tracker
cd STag-VR-FullBody-Tracker
cmake -B build
cmake --build build --config Release --target install
```

## SteamVR ドライバのインストール

ビルドまたはリリース zip の展開が完了したら、以下の手順で SteamVR にドライバを登録してください。

- あらかじめ SteamVR を一度起動しておいてください（`%localappdata%\openvr\openvrpaths.vrpath` が生成されている必要があります）。
- `install`（またはリリース zip の展開先）内の `driver_files` フォルダに移動し、そこにある
  ```
  driver_files\install_driver.bat
  ```
  を実行します。このバッチファイルはカレントディレクトリを基準に隣の `apriltagtrackers` フォルダを解決するため、必ず `driver_files` フォルダ内から（ダブルクリックするか、そのフォルダをカレントディレクトリにして）実行してください。
- スクリプトはドライバの登録に加えて、SteamVR 設定の `activateMultipleDrivers` を true に設定します。
- アンインストールする場合は、同じ `driver_files` フォルダ内の
  ```
  driver_files\uninstall_driver.bat
  ```
  を実行してください。

補足: 旧バージョンに存在した Python 製インストーラ `install_driver.exe` は廃止されました。古い手順に従って `install_driver.exe` を実行し「'apriltagtrackers' folder not found」と表示された場合は、代わりに上記の `install_driver.bat` を使用してください。

### トラブルシューティング

ビルド時に何か問題が発生した場合は、GitHub issue でお知らせください！

## 参考文献 / ソース
Benligiray, B., Topal, C., Akinlar, C. "STag: A stable fiducial marker system." Image and Vision Computing, 2019.

https://github.com/ManfredStoiber/stag

WxWidgets: A Cross-Platform GUI Library

https://github.com/wxWidgets/wxWidgets
