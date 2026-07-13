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
