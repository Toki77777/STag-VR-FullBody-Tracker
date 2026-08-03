# STag-VR-FullBody-Tracker

STag マーカーを使った VR 用フルボディトラッキング。

これは、fiducial マーカー（位置検出用マーカー）を使ってフルボディトラッキングシステムを作る試みです。これにより、スマホと段ボールだけで無料でフルボディトラッキングを実現できます。マーカーのサイズが 10cm 程度と小さく、640x480 解像度の PS Eye カメラでも、かなり良好なトラッキングが可能です。マーカーを大きくしたり、より高解像度で高速なスマホカメラを使ったりすれば、トラッキング精度はさらに向上します。

**注意: これは無料かつオープンソースのプロジェクトです。このために DRIVER4VR は必要ありません！**

使用するには、トラッカーを 3 つ作る必要があります — 両脚に 1 つずつ、腰に 1 つです。脚のトラッカーだけでは VRChat では動作しません！

このバージョンは、安定していてオクルージョン（遮蔽）に強い fiducial マーカーシステムである STag を採用しており、GUI やより分かりやすいキャリブレーションなど、システムを使いやすくするための多くの改良が加えられています。

### マーカーの印刷

[images-to-print](images-to-print) フォルダには、印刷してすぐに使える A4 の STag（**HD19**）マーカーシートが入っています。マーカーは 93mm（黒い正方形の辺の長さ）で、デフォルトのトラッカー構成に合わせてサイズと配置が調整されています。各ページには 1 つのトラッカーの表面（front）または裏面（back）のマーカーペアが配置されているため、トラッカーごとに表面ページ 1 枚と裏面ページ 1 枚を印刷してください（同じトラッカーの表面シートと裏面シートには、その同一トラッカーに属する異なるマーカー ID が印刷されています）。より多くのマーカーが必要な場合や、別サイズのマーカーが必要な場合は、[utilities/generate_stag_markers.py](utilities/generate_stag_markers.py) で自分で生成できます（numpy、opencv-python、pillow をインストールした Python が必要です）。

#### 白い余白を切り落とさないこと（重要）

STag は**黒い正方形の外形線**を追跡してマーカーを見つけます。黒い正方形の外側が明るくないと外形線が
存在せず、内側のパターンがどれだけ鮮明に写っていても **1 つも検出されません**。

- 各マーカーの周囲には、黒い正方形の一辺の **12.5% 以上**の白い余白が必要です（93mm なら約 12mm）。
  同梱シートはこの余白を含めて配置してあるので、**余白の内側で切り抜かないでください**。
- 切り抜いて貼る場合は、黒い正方形より上下左右それぞれ 12mm 以上大きい白い台紙に貼ってください。
- 2 つのマーカーの黒い正方形を隣接させないでください。互いの外形線が消えます。
- 余白が足りないマーカーを暗い服や暗い背景の前に持ってくると、黒同士が繋がって検出できなくなります。

### マーカーライブラリと ID の対応（重要）

**印刷したマーカーのライブラリと `config.yaml` の `markerLibrary` が一致していないと、1 つも検出されません。**
ID が同じでもライブラリが違えばパターンが全く違うため、見た目では区別できません。

ライブラリごとに使える ID の個数が決まっています。**この個数以上の ID は存在しないため、検出することは原理的に不可能です。**

| `markerLibrary` | ライブラリ | マーカー数 | 有効な ID |
| --- | --- | --- | --- |
| 0 | HD11 | 22309 | 0〜22308 |
| 1 | HD13 | 2884 | 0〜2883 |
| 2 | HD15 | 766 | 0〜765 |
| 3 | HD17 | 157 | 0〜156 |
| **4（既定）** | **HD19** | **38** | **0〜37** |
| 5 | HD21 | 12 | 0〜11 |
| 6 | HD23 | 6 | 0〜5 |

既定は HD19（38 個）で、`markersPerTracker: 12` と組み合わせて次のように割り当てられます。同梱の印刷シートもこの ID で生成されています。

| 用途 | ID 範囲 | 同梱シート |
| --- | --- | --- |
| トラッカー 0 | 0〜11（表 0,1 / 裏 2,3） | `trackers-a4-paper` 1 ページ目 |
| トラッカー 1 | 12〜23（表 12,13 / 裏 14,15） | 2 ページ目 |
| トラッカー 2 | 24〜35（表 24,25 / 裏 26,27） | 3 ページ目 |
| HMD 参照ボード | 36,37 | `reference-board-a4-paper` |

`markersPerTracker` × `trackerNum` がライブラリのマーカー数を超える場合は、収まる値に自動的に縮小し、その旨をログに出します（例: HD19 でトラッカー 3 個なら 12 個ずつ）。ID 範囲を明示指定していてライブラリの範囲外だった場合は、エラーをログに出して既定の割り当てに戻します。

HD11 など別のライブラリを使いたい場合は、`markerLibrary` を変更したうえで、そのライブラリでマーカーを再生成・再印刷してください。

```
python utilities/generate_stag_markers.py --hd 11 --sheet a4 --marker-size-mm 93 \
  --sheet-cols 1 --sheet-rows 2 --sheet-ids 0 1 45 46 90 91 --sheet-out front.pdf
```

プログラムは [releases](https://github.com/Toki77777/STag-VR-FullBody-Tracker/releases) タブからダウンロードできます。

![demo](images/demo.gif)

### 簡単なセットアップ動画:
近日公開

## キャリブレーションは初回の一度だけ

カメラキャリブレーション（charuco ボード）とマーカーキャリブレーションは、どちらも
`config/calib.yaml` に保存され、次回以降の起動で自動的に読み込まれます。
毎回やり直す必要はありません。

保存されているかどうかはウィンドウ下部のステータスバーで確認できます。

| 表示 | 意味 |
| --- | --- |
| `カメラ校正: 保存済み` | 前回のカメラキャリブレーションが読み込まれている |
| `カメラ校正: 未実施` | まだ一度も行われていない |
| `マーカー校正: 保存済み` | すべてのトラッカーのマーカーが揃っている |
| `マーカー校正: 1/3` | 3 つのうち 1 つ分しか保存されていない |

未実施の項目があるときは、起動時に次に行うべき手順だけを案内します。
両方が保存されていれば何も表示されず、そのままトラッキングを開始できます。

### 保存済みキャリブレーションの上書きについて

カメラキャリブレーションをやり直したとき、新しい結果が保存済みのものより明らかに悪い場合
（再投影誤差が大きい、撮影枚数が大幅に少ない、など）は、置き換える前に確認を出します。
両方の数値が表示されるので、保存済みのものを残すか置き換えるかを選べます。
数枚だけ撮って終えてしまい、以前の良いキャリブレーションを失うのを防ぐためです。

やり直しても構わない場合は OK を、以前のものを残す場合はキャンセルを押してください。

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

1. トラッカーが使っていない ID のマーカーを印刷します。既定（HD19、`markersPerTracker: 12`、
   トラッカー 3 個）ではトラッカーが 0〜35 を使うので、残りは **36,37** です。
   同梱の [images-to-print/reference-board-a4-paper](images-to-print/reference-board-a4-paper)
   がこの 2 枚です。**ライブラリのマーカー数を超える ID（HD19 なら 38 以上）は存在しないので使えません。**
2. 印刷したマーカーを HMD に固定します。**HMD に対して動かないこと**が条件です。ずれると、
   ずれた分がそのままトラッキング全体のずれになります。
3. `config.yaml` の `referenceMarker` を設定します。ID 範囲は `[markerIdBegin, markerIdEnd)`
   の半開区間で、有効化するときは必ず明示指定が必要です（暗黙の既定範囲は割り当てられません）。
   ```yaml
   referenceMarker:
      enabled: 1
      markerIdBegin: 36
      markerIdEnd: 38
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
