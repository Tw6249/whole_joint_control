# EID 鍙屽叧鑺傛帶鍒堕€熷害涓庡姏鐭╅渿鑽″垎鏋?
## 1. 闂涓庣粨璁?
鏈疄楠屾瘮杈冧袱绉?EID 鎺у埗娴嬭瘯锛?
1. **鍗曞叧鑺傛祴璇?*锛氬叾瀹冨叧鑺備繚鎸侀潤姝紝鍙帶鍒跺彸鑶濆叧鑺?RightKnee銆?2. **鍙屽叧鑺傛祴璇?*锛氬叾瀹冨叧鑺備繚鎸侀潤姝紝鍚屾椂鎺у埗鍙抽珛淇话鍏宠妭 RightHipPitch 涓庡彸鑶濆叧鑺?RightKnee銆?
瀹為獙鐜拌薄鏄細鍗曞叧鑺傚彸鑶濇帶鍒舵椂锛屼綅缃€侀€熷害銆佸姏鐭╅兘杈冨钩绋筹紱鑰屽彸楂嬩笌鍙宠啙鍚屾椂鎺у埗鏃讹紝浣嶇疆浠嶈兘澶ц嚧璺熻釜鍙傝€冿紝浣嗛€熷害鍜屽姏鐭╁嚭鐜板己鐑堥珮棰戦渿鑽°€?
鏍稿績缁撹濡備笅锛?
> 鍙屽叧鑺傛祴璇曚腑鐨勯€熷害鍜屽姏鐭╅渿鑽′笉鏄弬鑰冭建杩归€犳垚鐨勶紝鑰屾槸鐢扁€滅嫭绔嬪崟鍏宠妭 EID 妯″瀷鈥濆湪鈥滈珛-鑶濆己鑰﹀悎绯荤粺鈥濅腑澶遍厤瀵艰嚧銆傛ā鍨嬪け閰嶇粡 EID 瑙傛祴鍣ㄥ拰閫嗘ā鍨嬫斁澶э紝浣垮姏鐭╄繘鍏ラケ鍜岋紱鍚屾椂褰撳墠 `tau_slew_rate = 0`锛屾病鏈夊姏鐭╁彉鍖栫巼闄愬埗锛屽洜姝ら棴鐜舰鎴愰珮棰?chattering銆?
璇ョ粨璁虹敱涓夌被璇佹嵁鏀寔锛?
- 鍙傝€冮€熷害闈炲父灏忎笖骞虫粦锛屼絾瀹為檯閫熷害鍦ㄥ弻鍏宠妭娴嬭瘯涓ぇ骞呮斁澶с€?- 鍙屽叧鑺傛祴璇曚腑 EID 鍐呴儴瑙傛祴閲?`eta_dq` 鍜岃櫄鎷熺洰鏍?`r_d_q` 澶у箙鍋忕姝ｅ父鑼冨洿銆?- 鍙抽珛鍔涚煩闀挎湡澶勪簬鎺ヨ繎楗卞拰鐘舵€侊紝鍙宠啙鍔涚煩涔熸樉钁楀澶э紝骞朵笌閫熷害闇囪崱鍚岄銆?
## 2. 鏁版嵁鏉ユ簮涓庡鐜版柟寮?
娴嬭瘯鑴氭湰锛?
- [scripts/test_eid_right_leg_tracking.py](../scripts/test_eid_right_leg_tracking.py)

鍒嗘瀽鑴氭湰锛?
- [scripts/analyze_eid_joint_coupling.py](../scripts/analyze_eid_joint_coupling.py)
- [scripts/analyze_eid_control_dt_sweep.py](../scripts/analyze_eid_control_dt_sweep.py)
- [scripts/plot_eid_position_tracking.py](../scripts/plot_eid_position_tracking.py)

娴嬭瘯閰嶇疆锛?
- 鍗曞叧鑺傚彸鑶濋厤缃細[configs/right_knee_only_generated_eid_test_config.yaml](../configs/right_knee_only_generated_eid_test_config.yaml)
- 鍙抽珛 + 鍙宠啙閰嶇疆锛歔configs/right_hip_pitch_and_knee_generated_eid_test_config.yaml](../configs/right_hip_pitch_and_knee_generated_eid_test_config.yaml)
- 鎺у埗鍛ㄦ湡 sweep 閰嶇疆鐩綍锛歔configs/](../configs/)

鍘熷鏃ュ織锛?
- 鍗曞叧鑺傚彸鑶濇祴璇曪細[data/eid_right_leg_tests/right_knee_only/mujoco_closed_loop_log.csv](../data/eid_right_leg_tests/right_knee_only/mujoco_closed_loop_log.csv)
- 鍙抽珛 + 鍙宠啙娴嬭瘯锛歔data/eid_right_leg_tests/right_hip_pitch_and_knee/mujoco_closed_loop_log.csv](../data/eid_right_leg_tests/right_hip_pitch_and_knee/mujoco_closed_loop_log.csv)

鍒嗘瀽缁撴灉锛?
- 鎸囨爣琛細[data/eid_right_leg_tests/analysis/eid_coupling_metrics.csv](../data/eid_right_leg_tests/analysis/eid_coupling_metrics.csv)
- 鍥剧墖鐩綍锛歔data/eid_right_leg_tests/analysis/](../data/eid_right_leg_tests/analysis/)

澶嶇幇鍛戒护锛?
```bash
python scripts/test_eid_right_leg_tracking.py
python scripts/analyze_eid_joint_coupling.py
```

鍒嗘瀽鎸囨爣榛樿璺宠繃鍓?1 绉掑惎鍔ㄧ灛鎬侊紝缁熻绐楀彛涓?1.0s 鍒?15.0s銆?
## 3. 鍏抽敭鏁版嵁璇佹嵁

| case | joint | analysis_window_s | dq_ref_abs_max | dq_actual_abs_max | dq_actual_std | q_rmse | u_t_abs_mean | u_t_sat_90pct_frac | eta_dq_abs_mean | r_d_q_abs_max |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| single_knee | RightKnee | 1.0-15.0 | 0.0628326 | 0.0608538 | 0.042385 | 0.0096283 | 0.0401678 | 0.00% | 0.0100076 | 0.833492 |
| dual_hip_knee | RightHipPitch | 1.0-15.0 | 0.125665 | 2.62062 | 1.61987 | 0.013594 | 179.937 | 81.71% | 0.580819 | 3.74258 |
| dual_hip_knee | RightKnee | 1.0-15.0 | 0.0628326 | 4.80317 | 2.92225 | 0.0124712 | 145.504 | 0.00% | 0.692333 | 4.64672 |

浠庤〃涓彲浠ョ洿鎺ョ湅鍑猴細

1. **鍙傝€冮€熷害娌℃湁鍙樺ぇ**銆傚彸鑶濆湪鍗曞叧鑺傚拰鍙屽叧鑺傛祴璇曚腑浣跨敤鐩稿悓鍙傝€冿紝`dq_ref_abs_max = 0.0628 rad/s`銆?2. **瀹為檯閫熷害鏄捐憲鏀惧ぇ**銆傚彸鑶?`dq_actual_std` 浠?`0.0424 rad/s` 澧炲姞鍒?`2.922 rad/s`锛岀害涓哄崟鍏宠妭娴嬭瘯鐨?69 鍊嶃€?3. **鍔涚煩鏄捐憲鏀惧ぇ**銆傚彸鑶濆钩鍧囧姏鐭╀粠 `0.040 N路m` 澧炲姞鍒?`145.5 N路m`锛涘彸楂嬪钩鍧囧姏鐭╄揪鍒?`179.9 N路m`锛岀害涓?200 N路m 闄愬箙鐨?90%銆?4. **鍙抽珛闀挎湡楗卞拰**銆傚彸楂?`|u_t| > 90% tau_limit` 鐨勬瘮渚嬩负 `81.71%`銆?5. **瑙傛祴鍣ㄩ€熷害鎵板姩鏄捐憲澧炲ぇ**銆傚彸鑶?`mean |eta_dq|` 浠?`0.010` 澧炲姞鍒?`0.692`锛涘彸楂嬩负 `0.581`銆?6. **铏氭嫙鐩爣鍙樺緱涓嶅悎鐞?*銆傚彸鑶?`|r_d_q|` 鏈€澶ц揪鍒?`4.65 rad`锛岃繙楂樹簬瀹為檯鍙宠啙鍙傝€冭寖鍥?`0.65-0.85 rad`銆?
鍥犳锛岄棶棰樹笉鏄€滃弬鑰冩洸绾垮お婵€鐑堚€濓紝鑰屾槸 EID 鍐呴儴琛ュ伩閲忓湪鍙屽叧鑺傝€﹀悎涓嬭鏀惧ぇ锛屾渶鍚庤〃鐜颁负閫熷害鍜屽姏鐭╅珮棰戦渿鑽°€?
## 4. 鍥惧儚璇佹嵁

### 4.1 鍗曞叧鑺傚彸鑶濈ǔ瀹氭€ц瘉鎹?
![Single-joint EID stability evidence](../data/eid_right_leg_tests/analysis/fig0_single_knee_stability_evidence.png)

杩欏紶鍥惧彧浣跨敤鍗曞叧鑺傚彸鑶濇祴璇曠殑鏁版嵁锛岀伆鑹插尯鍩熸槸鍓?1 绉掑惎鍔ㄧ灛鎬侊紝鎸囨爣缁熻绐楀彛涓?1.0s 鍒?15.0s銆傚彲浠ョ湅鍒帮細

- 浣嶇疆 `q_actual` 骞虫粦璺熻釜 `q_ref`锛岀ǔ鎬?`q RMSE = 0.00963 rad`銆?- 閫熷害 `dq_actual` 涓?`dq_ref` 鍚岄噺绾э紝绋虫€?`std(dq) = 0.04239 rad/s`锛屾病鏈夊弻鍏宠妭娴嬭瘯涓殑楂橀澶у箙闇囪崱銆?- 鍔涚煩 `u_t` 鍑犱箮涓洪浂锛岀ǔ鎬?`mean |tau| = 0.04017 N路m`锛岃秴杩?90% 鍔涚煩闄愬箙鐨勬瘮渚嬩负 `0.00%`銆?- 瑙傛祴鍣ㄩ€熷害鎵板姩 `eta_dq` 寰堝皬锛岀ǔ鎬?`mean |eta_dq| = 0.01001`銆?
鍥犳锛屸€滃崟鍏宠妭骞崇ǔ鈥濅笉鏄彛澶村垽鏂紝鑰屾槸鐢变綅缃€侀€熷害銆佸姏鐭╁拰瑙傛祴鍣ㄥ唴閮ㄩ噺鍏卞悓鏀寔銆?
瀵瑰簲鏁版嵁鍜岄厤缃細

- 鏃ュ織锛歔data/eid_right_leg_tests/right_knee_only/mujoco_closed_loop_log.csv](../data/eid_right_leg_tests/right_knee_only/mujoco_closed_loop_log.csv)
- 娴嬭瘯閰嶇疆锛歔configs/right_knee_only_generated_eid_test_config.yaml](../configs/right_knee_only_generated_eid_test_config.yaml)
- 鍘熷杈撳嚭鍥撅細[data/eid_right_leg_tests/right_knee_only/right_knee_position_velocity_torque.png](../data/eid_right_leg_tests/right_knee_only/right_knee_position_velocity_torque.png)
- 鐢熸垚浠ｇ爜锛歔scripts/analyze_eid_joint_coupling.py](../scripts/analyze_eid_joint_coupling.py)

### 4.2 鍚屼竴涓彸鑶濆弬鑰冿細鍗曞叧鑺?vs 鍙屽叧鑺?
![Same RightKnee reference: single-joint vs hip+knee control](../data/eid_right_leg_tests/analysis/fig1_same_knee_single_vs_dual.png)

鍥句腑榛戣壊铏氱嚎鏄彸鑶濆弬鑰冦€傚彲浠ョ湅鍒帮細

- 鍗曞叧鑺傛祴璇曚腑锛屽彸鑶濆疄闄呬綅缃€侀€熷害鍜屽姏鐭╅兘杈冨钩绋炽€?- 鍙屽叧鑺傛祴璇曚腑锛屽彸鑶濅綅缃粛澶ц嚧璺熻釜鍙傝€冿紝浣嗛€熷害鍜屽姏鐭╁嚭鐜版槑鏄鹃珮棰戞尟鑽°€?- 鐢变簬鍙宠啙鍙傝€冨畬鍏ㄧ浉鍚岋紝鍥犳闇囪崱涓嶈兘褰掑洜浜庡彸鑶濆弬鑰冭建杩规湰韬€?
杩欏紶鍥捐瘉鏄庝簡绗竴鐐癸細**鍚屼竴涓彸鑶濆弬鑰冨湪鍙屽叧鑺傛帶鍒舵椂鎵嶆縺鍙戦渿鑽?*銆?
### 4.3 鍙屽叧鑺傚唴閮ㄨ瘖鏂細鍔涚煩楗卞拰涓庤娴嬪櫒鏀惧ぇ

![Dual-joint EID diagnostics](../data/eid_right_leg_tests/analysis/fig2_dual_internal_diagnostics.png)

杩欏紶鍥惧睍绀哄弻鍏宠妭娴嬭瘯涓殑 EID 鍐呴儴鍙橀噺锛?
- 绗竴琛岋細`u_star` 涓?`u_t`銆備袱鑰呴兘鍑虹幇鎺ヨ繎闄愬箙鐨勯珮棰戝垏鎹€?- 绗簩琛岋細`r_d_q` 涓?`q_ref`銆俙q_ref` 寰堝皬涓斿钩婊戯紝浣?`r_d_q` 琚斁澶у埌鏁?rad 閲忕骇銆?- 绗笁琛岋細`x_bar_dq` 涓?`eta_dq`銆傞€熷害瑙傛祴鐩稿叧閲忓嚭鐜颁笌鍔涚煩鍚岄鐨勯渿鑽°€?- 绗洓琛岋細`e_q` 涓?`e_dq`銆傚唴閮ㄨ宸鏀惧ぇ鍚庣洿鎺ラ┍鍔ㄥ姏鐭╄緭鍑恒€?
杩欏紶鍥捐瘉鏄庝簡绗簩鐐癸細**闇囪崱鏉ヨ嚜 EID 鍐呴儴琛ュ伩閾捐矾锛岃€屼笉鏄閮ㄥ弬鑰冭建杩?*銆?
### 4.4 缁熻鏌辩姸鍥撅細闇囪崱寮哄害鐨勫畾閲忚瘉鎹?
![Quantitative evidence of dual-joint oscillation](../data/eid_right_leg_tests/analysis/fig3_quantitative_bars.png)

璇ュ浘灏嗗崟鍏宠妭鍙宠啙銆佸弻鍏宠妭鍙抽珛銆佸弻鍏宠妭鍙宠啙鏀惧湪涓€璧锋瘮杈冿細

- `std(dq_actual)`锛氬弻鍏宠妭娴嬭瘯鏄捐憲楂樹簬鍗曞叧鑺傛祴璇曘€?- `mean |u_t| / tau_limit`锛氬崟鍏宠妭鍙宠啙鍑犱箮涓嶉渶瑕佸姏鐭╋紱鍙屽叧鑺傚彸楂嬬害涓?90%锛屽彸鑶濈害涓?49%銆?- `|u_t| > 90% limit`锛氬彸楂嬬害 82% 鏃堕棿鎺ヨ繎鍔涚煩闄愬箙銆?- `mean |eta_dq|`锛氬弻鍏宠妭娴嬭瘯涓殑閫熷害鎵板姩浼拌杩滃ぇ浜庡崟鍏宠妭娴嬭瘯銆?
杩欏紶鍥捐瘉鏄庝簡绗笁鐐癸細**鍙屽叧鑺傛帶鍒跺紩鍏ヤ簡寮烘壈鍔ㄤ及璁′笌澶у姏鐭╅ケ鍜?*銆?
### 4.5 灞€閮ㄦ斁澶у浘锛氶€熷害鍜屽姏鐭╁悓棰?chattering

![Zoomed dual-joint chattering window](../data/eid_right_leg_tests/analysis/fig4_zoomed_chattering.png)

鍦?8s 鍒?10s 鐨勫眬閮ㄧ獥鍙ｄ腑锛?
- 浣嶇疆鏇茬嚎浠嶇劧骞虫粦锛屾病鏈夋槑鏄鹃珮棰戣烦鍙樸€?- 閫熷害鏇茬嚎鍑虹幇蹇€熸璐熶氦鏇裤€?- 鍔涚煩鏇茬嚎涔熷嚭鐜板揩閫熸璐熶氦鏇匡紝骞朵笌閫熷害闇囪崱鍚岄銆?
杩欒鏄庨€熷害闇囪崱涓嶆槸浣嶇疆鍙傝€冨鑷寸殑鎱㈠彉鍖栵紝鑰屾槸鐢卞姏鐭╅棴鐜殑楂橀浜ゆ浛椹卞姩浜х敓銆?
### 4.6 鏈哄埗閾捐矾鍥?
![Theory chain](../data/eid_right_leg_tests/analysis/fig5_theory_chain.png)

鏈哄埗鍙互姒傛嫭涓猴細

```text
鍙屽叧鑺傚悓鏃惰繍鍔?  -> 楂?鑶濆姩鍔涘鑰﹀悎澧炲己
  -> 鍗曞叧鑺?EID 妯″瀷鏃犳硶瑙ｉ噴鑰﹀悎椤?  -> 瑙傛祴鍣ㄦ畫宸?eta_dq / x_bar_dq 澧炲ぇ
  -> 閫嗘ā鍨嬮€氳繃 1/dt 涓?1/dt^2 鏀惧ぇ璇樊
  -> u_star / u_t 杩涘叆楗卞拰
  -> 鏃犲姏鐭╁彉鍖栫巼闄愬埗鏃朵骇鐢熼珮棰?chattering
```

## 5. 鐞嗚鍒嗘瀽

### 5.1 鐪熷疄鍙屽叧鑺傜郴缁熸槸鑰﹀悎鍔ㄥ姏瀛?
瀵瑰彸楂嬩刊浠颁笌鍙宠啙涓や釜鍏宠妭锛岀湡瀹炴満鍣ㄤ汉鍔ㄥ姏瀛﹀彲鍐欎负锛?
```math
M(q)\ddot q + C(q, \dot q)\dot q + g(q) + J_c(q)^T\lambda = \tau
```

鍏朵腑锛?
- `q = [q_h, q_k]^T`锛屽垎鍒〃绀哄彸楂嬩刊浠颁笌鍙宠啙銆?- `M(q)` 鏄川閲忕煩闃点€?- `C(q, dq)dq` 鏄?Coriolis / centrifugal 椤广€?- `g(q)` 鏄噸鍔涢」銆?- `J_c(q)^T lambda` 鏄叾瀹冨叧鑺傝淇濇寔闈欐鎴栫害鏉熸椂寮曞叆鐨勭害鏉熷弽鍔涖€?
灞曞紑鍒扮 `i` 涓叧鑺傦細

```math
\tau_i = M_{ii}(q)\ddot q_i + M_{ij}(q)\ddot q_j
       + h_i(q, \dot q) + g_i(q) + [J_c(q)^T\lambda]_i
```

褰撳彧鍔ㄥ彸鑶濇椂锛屽彸楂嬭繎浼煎浐瀹氾紝`M_ij(q) qdd_j` 绛夎€﹀悎椤硅緝灏忥紝鍗曞叧鑺傝繎浼兼瘮杈冨鏄撴垚绔嬨€?
褰撳彸楂嬪拰鍙宠啙鍚屾椂杩愬姩鏃讹紝`M_ij(q) qdd_j`銆乣C(q,dq)dq` 鍜岃€﹀悎閲嶅姏椤归兘浼氬彉澶с€傛鏃舵瘡涓叧鑺傞兘涓嶈兘鍐嶇湅鎴愮嫭绔嬬殑涓€缁寸郴缁熴€?
### 5.2 褰撳墠鎺у埗鍣ㄤ娇鐢ㄧ殑鏄嫭绔嬪崟鍏宠妭妯″瀷

褰撳墠 EID 姣忎釜鍏宠妭浣跨敤鐙珛妯″瀷锛岀浉鍏充唬鐮佸湪 [include/eid_controller.hpp](../include/eid_controller.hpp)锛?
```math
\tau_i = J_{eff,i}\ddot q_i + b_i\dot q_i + A_i\sin(q_i) + B_i\cos(q_i) + \tau_{0,i}
```

璇ユā鍨嬫病鏈夊寘鍚細

- `M_ij(q) qdd_j` 闈炲瑙掓儻鎬ц€﹀悎锛?- `C_ij(q,dq) dq_j` 閫熷害鑰﹀悎锛?- 鍙屽叧鑺傚Э鎬佺浉鍏崇殑鐪熷疄閲嶅姏椤癸紱
- 鍏跺畠閿佸畾鍏宠妭閫犳垚鐨勭害鏉熷弽鍔涖€?
鍥犳锛屽湪鍙屽叧鑺傛祴璇曚腑锛岀湡瀹炲姩鍔涘涓庡崟鍏宠妭妯″瀷涔嬮棿瀛樺湪璇樊锛?
```math
\Delta_i = \tau_i^{real} - \tau_i^{single\ joint\ model}
```

褰?`q_h` 鍜?`q_k` 鍚屾椂鍙樺寲鏃讹紝`Delta_i` 鏄懆鏈熸€т笖杈冨ぇ鐨勶紱EID 瑙傛祴鍣ㄤ細鎶婅繖閮ㄥ垎璇樊瑙ｉ噴涓哄閮ㄦ壈鍔ㄣ€?
### 5.3 EID 瑙傛祴鍣ㄦ妸鑰﹀悎璇樊鏄犲皠鍒伴€熷害鎵板姩浼拌

鎺у埗鍣ㄥ唴閮ㄤ娇鐢?`eta_q` 鍜?`eta_dq` 淇鐘舵€侀娴嬨€傛牳蹇冮€昏緫浣嶄簬 [include/eid_controller.hpp:147-183](../include/eid_controller.hpp#L147-L183)锛?
```cpp
const double x_bar_q = x_hat_q + eta_q;
const double x_bar_dq = x_hat_dq + eta_dq;
...
const double tilde_x_q = q - x_bar_q;
const double tilde_x_dq = dq - x_bar_dq;
const double eta_next_dq =
    c.filter_alpha * c.observer_gain_dq * tilde_x_dq +
    (1.0 - c.filter_alpha) * eta_lpf_dq_;
```

瀹為獙涓細

- 鍗曞叧鑺傚彸鑶濓細`mean |eta_dq| = 0.010`
- 鍙屽叧鑺傚彸楂嬶細`mean |eta_dq| = 0.581`
- 鍙屽叧鑺傚彸鑶濓細`mean |eta_dq| = 0.692`

杩欒鏄庡弻鍏宠妭鎯呭喌涓嬶紝瑙傛祴鍣ㄧ‘瀹炴娴嬪埌浜嗘瘮鍗曞叧鑺傚ぇ鍑犲崄鍊嶇殑閫熷害娈嬪樊銆?
### 5.4 閫嗘ā鍨嬩负浠€涔堜細鍑虹幇 `dt^2`

鎺у埗鍣ㄩ€嗘ā鍨嬩綅浜?[include/eid_controller.hpp:254-283](../include/eid_controller.hpp#L254-L283)锛?
```cpp
const double tau_from_q =
    bias + model.Jeff * ((q_target_next - q - dt * dq) / (dt * dt));
const double tau_from_dq =
    bias + model.Jeff * ((dq_target_next - dq) / dt);
...
u_star = clamp(u_star, -model.tau_max, model.tau_max);
```

杩欓噷鐨勪袱涓?`dt` 涓嶆槸閲嶅鍐欓敊锛岃€屾槸鏉ヨ嚜绂绘暎杩愬姩瀛︼細

```math
q_{k+1} = q_k + dt\,\dot q_k + dt\,\Delta\dot q_k
```

鍙堝洜涓?
```math
\Delta\dot q_k = dt\,\ddot q_k
```

鎵€浠?
```math
q_{k+1} = q_k + dt\,\dot q_k + dt^2\,\ddot q_k
```

鍥犳鐢ㄤ笅涓€姝ヤ綅缃弽鎺ㄥ姞閫熷害鏃讹紝鑷劧寰楀埌

```math
\ddot q_k = \frac{q_{k+1} - q_k - dt\,\dot q_k}{dt^2}
```

杩欏氨鏄唬鐮侀噷 `(...)/(dt*dt)` 鐨勬潵婧愶細涓€涓?`dt` 鏉ヨ嚜閫熷害绉垎鍒颁綅缃紝鍙︿竴涓?`dt` 鏉ヨ嚜鍔犻€熷害绉垎鍒伴€熷害銆?
褰撳墠 `dt = 0.002s` 鏃讹細

```math
dt^2 = 4 \times 10^{-6}
```

鎵€浠ヤ綅缃娴嬭宸細琚?`1 / dt^2` 鏀惧ぇ銆傚浜庨珛鍏宠妭锛宍Jeff 鈮?1.0`锛岃嫢鍙湁 `0.001 rad` 鐨勪笅涓€姝ヤ綅缃娴嬭宸細

```math
J_{eff}\frac{0.001}{0.002^2}
\approx 250\ N\cdot m
```

杩欏凡缁忚秴杩囧彸楂嬬殑 `200 N路m` 鍔涚煩闄愬箙銆備篃灏辨槸璇达紝鍦?2 ms 鎺у埗鍛ㄦ湡涓嬶紝鏋佸皬鐨勬ā鍨嬭宸氨瓒充互瑙﹀彂楗卞拰銆?
### 5.5 楗卞拰鐨?`u_star` 浼氭妸铏氭嫙浣嶇疆鐩爣鎺ㄥ埌涓嶅悎鐞嗚寖鍥?
鍦?[include/eid_controller.hpp:164-173](../include/eid_controller.hpp#L164-L173)锛宍u_star` 琚浆鎹负铏氭嫙鐩爣锛?
```cpp
const double den = c.kp * c.kp + c.kd * c.kd;
const double w_q = c.kp / den;
const double w_dq = c.kd / den;
const double r_d_q = ref.now.q + w_q * inv.u_star;
const double r_d_dq = ref.now.dq + w_dq * inv.u_star;
```

褰撳墠鑵块儴鍙傛暟鏉ヨ嚜 [data/eid_right_leg_tests/right_hip_pitch_and_knee/generated_eid_test_config.yaml:54-60](../data/eid_right_leg_tests/right_hip_pitch_and_knee/generated_eid_test_config.yaml#L54-L60)锛?
```yaml
kp: 60.0
kd: 10.0
observer_gain_q: 0.8
observer_gain_dq: 0.5
filter_alpha: 0.7
```

鍥犳锛?
```math
w_q = \frac{60}{60^2 + 10^2}
    = \frac{60}{3700}
    \approx 0.0162
```

鑻?`u_star` 楗卞拰锛?
- 鍙抽珛 `u_star = 200 N路m` 鏃讹紝`r_d_q` 鍋忕Щ绾?`3.24 rad`銆?- 鍙宠啙 `u_star = 300 N路m` 鏃讹紝`r_d_q` 鍋忕Щ绾?`4.86 rad`銆?
杩欎笌鏁版嵁涓€鑷达細

- 鍙抽珛 `|r_d_q|max = 3.74 rad`
- 鍙宠啙 `|r_d_q|max = 4.65 rad`

鑰岀湡瀹炲弬鑰冭寖鍥村彧鏈夛細

- 鍙抽珛锛氱害 `-0.5` 鍒?`-0.1 rad`
- 鍙宠啙锛氱害 `0.65` 鍒?`0.85 rad`

鎵€浠ワ紝鍐呴儴铏氭嫙鐩爣宸茬粡杩滆繙鍋忕鐪熷疄鍙傝€冦€傛帴涓嬫潵 PD 璇樊浼氱户缁妸 `u_raw` 鎺ㄥ悜楗卞拰銆?
### 5.6 鏃犲姏鐭╁彉鍖栫巼闄愬埗瀵艰嚧 chattering

鍔涚煩闄愬埗閫昏緫浣嶄簬 [include/eid_controller.hpp:241-249](../include/eid_controller.hpp#L241-L249)锛?
```cpp
const double tau_limit = std::min(std::abs(c.tau_limit), cfg_.plant.tau_max);
double limited = clamp(tau, -tau_limit, tau_limit);
const double slew = c.tau_slew_rate;
if (slew > 0.0) {
    const double max_delta = slew * std::max(dt, 0.0);
    limited = clamp(limited, last_tau_ - max_delta, last_tau_ + max_delta);
}
```

浣嗘祴璇曢厤缃腑 [data/eid_right_leg_tests/right_hip_pitch_and_knee/generated_eid_test_config.yaml:44-48](../data/eid_right_leg_tests/right_hip_pitch_and_knee/generated_eid_test_config.yaml#L44-L48)锛?
```yaml
policy_dt: 0.05
startup_blend_duration_s: 0.0
tau_slew_rate: 0
```

鍥犳 `u_t` 鍙仛骞呭€奸檺骞咃紝涓嶉檺鍒剁浉閭绘帶鍒跺懆鏈熶箣闂寸殑鍙樺寲銆傚綋鍐呴儴璇樊鏀瑰彉绗﹀彿鏃讹紝鍔涚煩鍙互鍦ㄧ浉閭诲懆鏈熷唴浠庢闄愬箙璺冲埌璐熼檺骞咃紝浠庤€屽舰鎴愰珮棰?chattering銆?
### 5.7 鏇村ぇ鐨勬帶鍒跺懆鏈熷苟涓嶈兘缂撹В鍙屽叧鑺傞渿鑽?
涓轰簡妫€楠屸€滃澶ф帶鍒跺懆鏈熸槸鍚︿細鍓婂急 `1/dt^2` 鏀惧ぇ鈥濓紝鍙堝仛浜嗕竴涓?sweep锛?*鐗╃悊绉垎姝ラ暱淇濇寔 2 ms 涓嶅彉锛屽彧鎶?EID 鎺у埗鍛ㄦ湡鏀规垚 4 ms銆?0 ms銆?0 ms**銆傚畬鏁寸粨鏋滆 [data/eid_right_leg_tests_dt_sweep/analysis/](../data/eid_right_leg_tests_dt_sweep/analysis/)锛屽搴旇剼鏈槸 [scripts/analyze_eid_control_dt_sweep.py](../scripts/analyze_eid_control_dt_sweep.py)銆?
鍏堢湅鏈€鍏抽敭鐨勭粨璁猴細

- **鍗曡啙娴嬭瘯浠嶇劧绋冲畾**锛屼絾鎺у埗鍛ㄦ湡瓒婂ぇ锛岃宸暐寰笂鍗囥€?- **鍙屽叧鑺傛祴璇曟槑鏄惧彉宸?*锛氬彸楂嬪拰鍙宠啙鐨勯€熷害闇囪崱骞呭害閮介殢鐫€鎺у埗鍛ㄦ湡澧炲ぇ鑰屾樉钁椾笂鍗囥€?- **鏇村ぇ鐨?dt 骞舵病鏈夆€滄姂鍒堕渿鑽♀€?*锛岃€屾槸鍑忓皯浜嗗弽棣堟洿鏂伴鐜囷紝璁╄€﹀悎璇樊鍦ㄦ洿闀跨殑寮€鐜尯闂撮噷绉疮銆?
#### 鍙屽叧鑺傛祴璇曠殑瀹氶噺缁撴灉

| dt_s | joint | q_rmse | dq_actual_std | u_t_abs_mean | eta_dq_abs_mean |
| --- | --- | --- | --- | --- | --- |
| 0.004 | RightHipPitch | 0.0458 | 3.2688 | 179.5 | 1.1671 |
| 0.010 | RightHipPitch | 0.1870 | 7.8660 | 177.4 | 2.8031 |
| 0.020 | RightHipPitch | 0.3183 | 12.2657 | 170.9 | 4.1021 |
| 0.004 | RightKnee | 0.0258 | 5.8132 | 142.1 | 1.3852 |
| 0.010 | RightKnee | 0.1451 | 14.5204 | 152.5 | 3.2208 |
| 0.020 | RightKnee | 0.4039 | 20.3123 | 113.8 | 4.2830 |

杩欑粍鏁版嵁璇存槑锛?
1. `q_rmse` 闅忔帶鍒跺懆鏈熷澶ф槑鏄炬伓鍖栵紝灏ゅ叾鏄彸鑶濆湪 20 ms 鏃跺凡缁忎笂鍗囧埌 `0.404 rad`銆?2. `dq_actual_std` 鎸佺画澧炲ぇ锛屽彸鑶濅粠 `5.81 rad/s` 澧炲埌 `20.31 rad/s`锛屽彸楂嬩粠 `3.27 rad/s` 澧炲埌 `12.27 rad/s`銆?3. `eta_dq_abs_mean` 涔熼殢 dt 澧炲ぇ鑰屼笂鍗囷紝璇存槑鏇撮暱鐨勬帶鍒堕棿闅旇妯″瀷澶遍厤鍜岃€﹀悎璇樊绱Н寰楁洿涓ラ噸銆?4. `u_t_abs_mean` 浠嶇劧缁存寔鍦ㄥ緢楂樼殑姘村钩锛岃鏄庨棶棰樹笉鏄€滃姏鐭╁お灏忔墍浠ユ病鏁堟灉鈥濓紝鑰屾槸闂幆鏈韩鍦ㄧ敤澶у姏鐭╄拷閫愰敊璇殑鍐呴儴鐩爣銆?
瀵瑰簲鐨勫浘濡備笅锛?
![Control-dt sweep metrics](../data/eid_right_leg_tests_dt_sweep/analysis/fig1_dt_sweep_metrics.png)

![RightHipPitch across control dt values](../data/eid_right_leg_tests_dt_sweep/analysis/fig2_dt_sweep_right_hip_pitch.png)

![RightKnee across control dt values](../data/eid_right_leg_tests_dt_sweep/analysis/fig3_dt_sweep_right_knee.png)

浠庣悊璁轰笂鐪嬶紝杩欎釜鐜拌薄骞朵笉鐭涚浘锛?
- `dt` 鍙樺ぇ鏃讹紝`1/dt^2` 纭疄鍙樺皬锛?- 浣嗘帶鍒跺櫒鏇存柊棰戠巼涔熷彉浣庯紝绯荤粺鍦ㄦ瘡娆℃洿鏂颁箣闂磋闈犳洿闀挎椂闂寸殑寮€鐜紨鍖栵紱
- 鍦ㄥ己鑰﹀悎绯荤粺閲岋紝妯″瀷璇樊銆佽娴嬪櫒璇樊鍜岀害鏉熷弽鍔涗細鍦ㄦ洿闀跨殑鏃堕棿绐楅噷绉疮锛?- 鍥犳锛?*鍑忓皯閫嗘ā鍨嬪鐩婄殑鍚屾椂锛屼篃闄嶄綆浜嗛棴鐜籂閿欓鐜?*銆?
鍦ㄦ湰瀹為獙閲岋紝鍚庤€呮槸涓诲鏁堝簲锛屾墍浠ユ帶鍒跺懆鏈熶粠 4 ms 澧炲姞鍒?10 ms銆?0 ms 鍚庯紝鍙屽叧鑺傞渿鑽″弽鑰屾洿涓ラ噸銆?
## 6. 涓轰粈涔堜綅缃湅璧锋潵杩樿兘璺熻釜锛岃€岄€熷害鍜屽姏鐭╁緢宸紵

杩欏苟涓嶇煕鐩俱€?
浣嶇疆鏄綆棰戦噺锛屼笖鍙傝€冩槸 0.1 Hz 鐨勬參閫熸寮︺€傚嵆浣垮姏鐭╁嚭鐜伴珮棰戞璐熶氦鏇匡紝浣嶇疆缁忚繃鍔ㄥ姏瀛︾Н鍒嗗悗浠嶅彲鑳借〃鐜颁负杈冨钩婊戠殑浣庨璺熻釜銆?
閫熷害鍜屽姏鐭╁垯鐩存帴鍙嶆槧楂橀闂幆琛屼负锛?
- 鍔涚煩 `u_t` 鏄帶鍒跺櫒鐩存帴杈撳嚭锛屾渶鍏堝嚭鐜伴ケ鍜屽拰姝ｈ礋鍒囨崲銆?- 閫熷害 `dq_actual` 鏄姏鐭╅┍鍔ㄤ笅鐨勪竴闃剁Н鍒嗗搷搴旓紝瀵归珮棰戝姏鐭╂洿鏁忔劅銆?- 浣嶇疆 `q_actual` 鏄€熷害鍐嶆绉垎鍚庣殑缁撴灉锛岄珮棰戞垚鍒嗚杩涗竴姝ュ钩婊戙€?
鍥犳锛屸€滀綅缃?RMSE 涓嶅ぇ鈥濅笉鑳借鏄庢帶鍒舵槸鍋ュ悍鐨勩€傝瀹為獙涓湡姝ｇ殑闂鏄細鎺у埗鍣ㄧ敤寰堝ぇ鐨勯珮棰戝姏鐭╂潵缁存寔涓€涓湅浼艰繕鍙互鐨勪綅缃宸紝杩欏鐪熷疄鏈哄櫒浜烘槸涓嶅畨鍏ㄧ殑銆?
## 7. 缁撹

鏍规嵁鏁版嵁銆佸浘鍜岀悊璁哄垎鏋愶紝鍙互寰楀埌濡備笅鍒ゆ柇锛?
1. **闇囪崱涓嶆槸鐢卞弬鑰冭建杩归€犳垚鐨?*銆傚彸鑶濆弬鑰冨湪鍗曞叧鑺傚拰鍙屽叧鑺傛祴璇曚腑鐩稿悓锛屼絾鍙屽叧鑺傛祴璇曟墠鍑虹幇閫熷害涓庡姏鐭╅渿鑽°€?2. **闇囪崱鏄敱鍙屽叧鑺傚姩鍔涘鑰﹀悎瑙﹀彂鐨?*銆傚綋鍓?EID 妯″瀷鏄嫭绔嬪崟鍏宠妭妯″瀷锛屾棤娉曡В閲婂彸楂嬩笌鍙宠啙鍚屾椂杩愬姩鏃剁殑闈炲瑙掓儻鎬с€侀€熷害鑰﹀悎鍜岄噸鍔涜€﹀悎銆?3. **EID 瑙傛祴鍣ㄦ娴嬪埌澶ф畫宸悗锛岄€嗘ā鍨嬭繘涓€姝ユ斁澶ц宸?*銆俙eta_dq` 鍦ㄥ弻鍏宠妭娴嬭瘯涓瘮鍗曞叧鑺傚ぇ鍑犲崄鍊嶏紱2 ms 鎺у埗鍛ㄦ湡涓殑 `1/dt` 涓?`1/dt^2` 椤逛娇寰堝皬鐨勯娴嬭宸彉鎴愬緢澶х殑鍔涚煩璇锋眰銆?4. **鍔涚煩楗卞拰鍜屾棤 slew 闄愬埗鍏卞悓瀵艰嚧 chattering**銆傚彸楂?81.71% 鏃堕棿鎺ヨ繎 90% 浠ヤ笂鍔涚煩闄愬箙锛沗tau_slew_rate = 0` 鍏佽鍔涚煩鍦ㄧ浉閭诲懆鏈熷唴澶у箙璺冲彉銆?5. **浣嶇疆璺熻釜琛ㄩ潰姝ｅ父涓嶄唬琛ㄦ帶鍒剁ǔ瀹?*銆傛湰瀹為獙涓綅缃宸皬锛屼絾閫熷害銆佸姏鐭╁拰鍐呴儴铏氭嫙鐩爣宸茬粡鏄庢樉寮傚父銆?
## 8. 寤鸿鐨勬敼杩涙柟鍚?
寤鸿鎸変粠浣庨闄╁埌楂樻敹鐩婄殑椤哄簭鏀硅繘锛?
1. **鍏堝惎鐢ㄥ姏鐭╁彉鍖栫巼闄愬埗**
   缁欐祴璇曢厤缃缃悎鐞嗙殑 `tau_slew_rate`锛岄伩鍏嶆璐熼檺骞呬箣闂村揩閫熻烦鍙樸€?
2. **闄嶄綆閫熷害鎵板姩瑙傛祴鍣ㄥ鐩?*
   灏濊瘯鍑忓皬 `observer_gain_dq` 鍜?`filter_alpha`锛岃瀵?`eta_dq` 涓?`u_t` 鏄惁闄嶄綆銆?
3. **闄嶄綆閫嗘ā鍨嬫縺杩涚▼搴?*
   璋冩暣 `inverse_q_weight` / `inverse_dq_weight`锛屽噺灏?`1/dt^2` 浣嶇疆璇樊椤瑰 `u_star` 鐨勫奖鍝嶃€?
4. **闄愬埗铏氭嫙鐩爣鍋忕Щ**
   瀵?`r_d_q` 鍜?`r_d_dq` 澧炲姞鐩稿浜庣湡瀹炲弬鑰冪殑鍋忕Щ闄愬埗锛岄槻姝?`u_star` 楗卞拰鍚庢妸铏氭嫙鐩爣鎺ㄥ埌鏁?rad 涔嬪銆?
5. **鏋勫缓浜岀淮鑰﹀悎 EID 妯″瀷**
   瀵瑰彸楂?+ 鍙宠啙寤虹珛 2-DoF 妯″瀷锛?
   ```math
   \tau = M(q)\ddot q + C(q,\dot q)\dot q + g(q)
   ```

   杩欐牱鍙互鏄惧紡寤烘ā `M_hk`銆乣M_kh` 绛夎€﹀悎椤癸紝鑰屼笉鏄涓や釜鍗曞叧鑺?EID 浜掔浉鎶婂鏂圭殑鍔ㄥ姏瀛﹀奖鍝嶅綋鎴愭壈鍔ㄣ€?
6. **鐢ㄦ洿鐗╃悊鐨勬柟寮忎繚鎸佸叾瀹冨叧鑺傞潤姝?*
   褰撳墠娴嬭瘯涓轰簡闅旂鐩爣鍏宠妭锛屼細姣忔閿佸畾闈炵洰鏍囧叧鑺傜姸鎬併€傚悗缁彲浠ユ敼鎴愰珮闃诲凹 PD hold 鎴?MuJoCo equality constraint锛屼互鏇存帴杩戠湡瀹炵墿鐞嗙害鏉熴€?
## 9. 闄勫綍锛氱敓鎴愮殑鏂囦欢

- 璇婃柇鑴氭湰锛歔scripts/analyze_eid_joint_coupling.py](../scripts/analyze_eid_joint_coupling.py)
- 鎺у埗鍛ㄦ湡 sweep 鑴氭湰锛歔scripts/analyze_eid_control_dt_sweep.py](../scripts/analyze_eid_control_dt_sweep.py)
- 娴嬭瘯鑴氭湰锛歔scripts/test_eid_right_leg_tracking.py](../scripts/test_eid_right_leg_tracking.py)
- 浣嶇疆璺熻釜缁樺浘鑴氭湰锛歔scripts/plot_eid_position_tracking.py](../scripts/plot_eid_position_tracking.py)
- 鍗曞叧鑺傚彸鑶濋厤缃細[configs/right_knee_only_generated_eid_test_config.yaml](../configs/right_knee_only_generated_eid_test_config.yaml)
- 鍙抽珛 + 鍙宠啙閰嶇疆锛歔configs/right_hip_pitch_and_knee_generated_eid_test_config.yaml](../configs/right_hip_pitch_and_knee_generated_eid_test_config.yaml)
- 鎺у埗鍛ㄦ湡 sweep 閰嶇疆鐩綍锛歔configs/](../configs/)
- 鎸囨爣 CSV锛歔data/eid_right_leg_tests/analysis/eid_coupling_metrics.csv](../data/eid_right_leg_tests/analysis/eid_coupling_metrics.csv)
- 鎸囨爣 Markdown 琛細[data/eid_right_leg_tests/analysis/eid_coupling_metrics_table.md](../data/eid_right_leg_tests/analysis/eid_coupling_metrics_table.md)
- dt sweep 鎸囨爣 CSV锛歔data/eid_right_leg_tests_dt_sweep/analysis/eid_dt_sweep_metrics.csv](../data/eid_right_leg_tests_dt_sweep/analysis/eid_dt_sweep_metrics.csv)
- dt sweep Markdown 琛細[data/eid_right_leg_tests_dt_sweep/analysis/eid_dt_sweep_metrics_table.md](../data/eid_right_leg_tests_dt_sweep/analysis/eid_dt_sweep_metrics_table.md)
- 鍗曞叧鑺傜ǔ瀹氭€у浘锛歔data/eid_right_leg_tests/analysis/fig0_single_knee_stability_evidence.png](../data/eid_right_leg_tests/analysis/fig0_single_knee_stability_evidence.png)
- 鍗曞叧鑺傚師濮嬭緭鍑哄浘锛歔data/eid_right_leg_tests/right_knee_only/right_knee_position_velocity_torque.png](../data/eid_right_leg_tests/right_knee_only/right_knee_position_velocity_torque.png)
- 鍥?1锛歔data/eid_right_leg_tests/analysis/fig1_same_knee_single_vs_dual.png](../data/eid_right_leg_tests/analysis/fig1_same_knee_single_vs_dual.png)
- 鍥?2锛歔data/eid_right_leg_tests/analysis/fig2_dual_internal_diagnostics.png](../data/eid_right_leg_tests/analysis/fig2_dual_internal_diagnostics.png)
- 鍥?3锛歔data/eid_right_leg_tests/analysis/fig3_quantitative_bars.png](../data/eid_right_leg_tests/analysis/fig3_quantitative_bars.png)
- 鍥?4锛歔data/eid_right_leg_tests/analysis/fig4_zoomed_chattering.png](../data/eid_right_leg_tests/analysis/fig4_zoomed_chattering.png)
- 鍥?5锛歔data/eid_right_leg_tests/analysis/fig5_theory_chain.png](../data/eid_right_leg_tests/analysis/fig5_theory_chain.png)
- dt sweep 鍥?1锛歔data/eid_right_leg_tests_dt_sweep/analysis/fig1_dt_sweep_metrics.png](../data/eid_right_leg_tests_dt_sweep/analysis/fig1_dt_sweep_metrics.png)
- dt sweep 鍥?2锛歔data/eid_right_leg_tests_dt_sweep/analysis/fig2_dt_sweep_right_hip_pitch.png](../data/eid_right_leg_tests_dt_sweep/analysis/fig2_dt_sweep_right_hip_pitch.png)
- dt sweep 鍥?3锛歔data/eid_right_leg_tests_dt_sweep/analysis/fig3_dt_sweep_right_knee.png](../data/eid_right_leg_tests_dt_sweep/analysis/fig3_dt_sweep_right_knee.png)
