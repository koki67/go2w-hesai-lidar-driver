# Plan

- Created: 2026-07-22T17:18:30+09:00
- Snapshot: 2026-07-22T17:18:30+09:00
- Status: final
- Session: unavailable
- Branch: main (implementation branch to create: `fix/prevent-stale-packet-replay`)
- Workspace: /home/user/ws/go2w-hesai-lidar-driver
- Scope: Fix the PandarXT-16 stale-packet replay defect in the shared `koki67/go2w-hesai-lidar-driver`, add deterministic regression and defensive timestamp/sequence checks, validate it through `go2w-experiment-recorder` and `dlio-go2w`, then update every confirmed direct dependency pin. Preserve the existing ROS package, topic, frame, message, and launch contracts.

## Context and current state

- The primary repository is `/home/user/ws/go2w-hesai-lidar-driver`, currently clean on `main` at `6c6a93aa3c844a49d2cb26360e5bdbc4687b427d` and tracking `origin/main` at `https://github.com/koki67/go2w-hesai-lidar-driver.git`.
- GitHub reports this as an independent repository rather than a GitHub fork. Its README states that it is a ROS 2 port of Hesai's legacy `HesaiLidar_General_ROS` driver.
- The observed failure is that an XT16 point cloud occasionally contains data approximately 7 seconds old. Historical measurements showed additional ages around 7.165 and 7.172 seconds.
- The strongest root-cause evidence is the shared `PacketsBuffer` implementation in `src/HesaiLidar_General_SDK/src/PandarGeneralRaw/src/pandarGeneral_internal.h`: it stores 36,000 packets, exposes raw producer and consumer iterators across threads without synchronization, increments the producer iterator to `end()` before wrapping it to `begin()`, and performs a separate `hasEnoughPackets()`/read/advance sequence on the consumer. The consumer can therefore observe the transient producer `end()` state, wrap itself, and read the oldest slot before the producer overwrites it.
- PandarXT-16 dual return produces approximately 5,000 UDP packets per second: 640,000 points/s divided by 8 blocks x 16 points/packet. One complete 36,000-packet ring is therefore approximately 7.2 seconds, matching the anomaly.
- `ProcessLiarPacket()` currently performs a time-of-check/time-of-use read using `hasEnoughPackets()`, `getIterCalc()`, and `moveIterCalc()`. `PushLiDARData()` ignores the queue result. The queue also busy-waits above `MAX_ITERATOR_DIFF` (400), which complicates shutdown if the consumer exits while the producer is waiting.
- XT packets include a four-byte UDP sequence at the end of the packet, but `ParseXTData()` currently stops after the timestamp and does not expose or validate the sequence.
- With the default `timestamp_type=realtime`, each point receives the packet reception timestamp. `EmitBackMessege()` derives the cloud timestamp from the first point, and `src/main_ros2.cc` publishes it without a monotonicity check. A stale packet can therefore make a whole cloud header regress or inject stale point timestamps into an otherwise current cloud.
- The package currently has no ROS-integrated unit tests. The SDK's existing `test/test.cc` is a manual PCAP example and is not a regression test.
- Confirmed direct consumers all pin the same old commit `6c6a93a`:
  - `/home/user/ws/go2w-experiment-recorder` via submodule.
  - `/home/user/ws/dlio-go2w` via submodule.
  - `/home/user/ws/slam-go2w` via submodule.
  - `/home/user/ws/wlo-go2w` via submodule.
- `/home/user/ws/frontier-next/dependencies.repos` pins the same commit and its vendor-patch manifest, dependency policy, and content-addressed patch path also bind to that base commit. The many `frontier-next-*` directories are worktrees of this repository, not independent dependency owners.
- `/home/user/ws/nav-frontier-go2w-v3/humble_ws/src/hesai_lidar` is a tracked source copy rather than a submodule, so it requires an explicit synchronized source update or replacement.
- The `.gitmodules` files in `slam-go2w` and `wlo-go2w` contain the malformed URL `git.com:koki67/go2w-hesai-lidar-driver.git`, although their existing local submodule remotes have been corrected to the HTTPS GitHub URL.
- At snapshot time the primary driver and four direct submodule consumers were clean. `frontier-next` was clean but one commit behind its remote; `nav-frontier-go2w-v3` was clean on `feat/terrain-costmap-stair-climb`. All repositories must be rechecked before edits because these facts can change.

## Decisions and constraints

- Fix the currently shared driver first. Do not combine this defect fix with migration to `HesaiLidar_ROS_2.0`; that migration has broader configuration, message, point-field, and runtime compatibility risk and should be evaluated separately.
- Preserve the public runtime contract: ROS package `hesai_lidar`, executable `hesai_lidar_node`, `/points_raw` and raw-packet topics, `hesai_lidar` frame semantics, current PointCloud2 fields, launch argument names, and defaults.
- Replace raw shared iterators with a bounded single-producer/single-consumer queue whose publication and consumption ordering is defined by C++ atomics. Do not retain `hasEnoughPackets()` followed by a separate raw-slot access.
- Keep the existing storage size available for compatibility but enforce the intended maximum backlog of 400 packets. A producer must never spin indefinitely. On overload, reject/drop a packet, increment a counter, and emit throttled diagnostics; keeping the driver responsive and bounded is more important than accumulating seconds of latency.
- Use acquire/release ordering: the producer writes a slot before release-publishing its write index; the consumer acquire-loads the write index before copying the slot and release-publishes its read index. Only the producer writes the write index and only the consumer writes the read index.
- Treat queue correctness as the root fix. XT UDP sequence and timestamp checks are defense-in-depth and observability, not substitutes for the queue fix.
- For XT, decode the final four-byte little-endian sequence for all supported XT packet sizes. Handle normal increment, forward gaps, uint32 rollover, duplicates, and backward/reordered packets. Drop duplicates and backward packets; count forward gaps. Reset sequence baseline on driver restart and after a documented long receive gap so a LiDAR reboot can recover instead of being rejected forever.
- Reject a packet whose reception timestamp regresses substantially relative to the last accepted packet, with a tolerance small compared with 7 seconds but large enough for normal floating-point jitter. Centralize this decision in a testable helper and log the measured regression. Reset the guard on Start/Stop.
- Before publishing a point cloud, reject empty clouds and clouds whose header timestamp regresses beyond the same documented tolerance. This is a final safety boundary for downstream SLAM/LIO. Log and count drops rather than silently hiding them.
- Diagnostics in this targeted fix will be throttled ROS log summaries and shutdown totals, avoiding a new public `diagnostic_msgs` topic contract. Counters must cover queue overload, sequence gaps, duplicates/backward packets, packet timestamp regressions, and cloud timestamp regressions.
- Add deterministic tests that require no LiDAR hardware. Hardware and long-duration checks remain mandatory before claiming the physical defect resolved; if hardware is unavailable, report those gates as UNVERIFIED.
- Do not repair historical bags by rewriting them. The source fix prevents new corrupt data; old bags remain unchanged and require a separate filtering workflow if they must be reused.
- Use semantic branches without a `codex/` prefix. Preserve unrelated user changes and do not update a dirty consumer repository without first resolving scope with the user.
- Commit and push the driver fix only after offline build/tests pass. Update consumer pins only to the resulting immutable full commit SHA. Do not rely on the submodule `branch = main` field, because ordinary `git submodule update` resolves the parent gitlink rather than automatically following the branch.

## Final plan

1. Re-enter implementation in a fresh context.
   - Read all applicable `AGENTS.md` files and this entire saved plan before editing.
   - Recheck `git status`, current branch, remotes, HEADs, submodule state, and GitHub-visible driver state.
   - Verify that no newer upstream or local fix has already changed `PacketsBuffer`, XT parsing, or publishing semantics.
   - Create `/home/user/ws/go2w-hesai-lidar-driver` branch `fix/prevent-stale-packet-replay` from the verified current `origin/main` without disturbing unrelated work.

2. Isolate and implement the bounded SPSC queue.
   - Add a small reusable internal header under `src/HesaiLidar_General_SDK/src/PandarGeneralRaw/src/` for a templated fixed-capacity SPSC queue so it can be unit-tested without constructing the ROS/PCL driver.
   - Store fixed slots plus atomic read/write indices; reserve one slot to distinguish full from empty; provide a single atomic `try_push(const T&)` and `try_pop(T&)` contract, `reset()`, an approximate depth for diagnostics, and no public raw iterators.
   - Represent the intended 400-packet maximum backlog explicitly instead of using iterator subtraction across wrap. Return failure immediately when the permitted backlog is full; never busy-wait in the queue.
   - Replace `PacketsBuffer`, `hasEnoughPackets()`, `getIterCalc()`, and `moveIterCalc()` in `pandarGeneral_internal.h/.cc` with the new queue API.
   - Make `ProcessLiarPacket()` use one `try_pop()` operation so availability check and packet copy cannot race. Make `PushLiDARData()` handle `try_push()` failure, update counters, and issue throttled warnings.
   - Reset the queue and all packet-order state at a safe lifecycle point before threads start. Ensure Stop cannot leave the producer blocked after the consumer exits.

3. Add XT sequence parsing and packet-order defenses.
   - Add a testable helper for decoding the trailing four-byte little-endian XT sequence without changing the public ROS messages or point type.
   - Add an internal packet-order tracker with explicit outcomes for first packet, in-order packet, forward gap, duplicate, backward/reordered packet, uint32 rollover, and restart-after-idle.
   - Apply it only to supported XT/XT16/XTM packet layouts after packet size and start-of-packet validation. Count and log forward gaps; drop duplicates and backward/reordered packets before point calculation.
   - Reset the tracker on Start/Stop and after a documented idle/restart threshold. Verify that sequence rollover from `0xffffffff` to `0` is accepted.
   - Add a testable packet reception timestamp guard. Drop a packet with a significant backward reception-time jump before it can alter azimuth/frame state or add points. Keep minor jitter within the documented tolerance accepted.

4. Add the final cloud-publication guard and diagnostics.
   - In `src/main_ros2.cc`, retain the current publisher QoS and topic contract initially.
   - Reject an empty PCL cloud before indexing its first point.
   - Track the last published cloud timestamp and reject a regression beyond the documented tolerance. Do not reorder clouds, because a cloud can contain mixed current/stale points and reordering would publish corrupted geometry later.
   - Add throttled `RCLCPP_WARN` messages containing event type, measured delta or sequence values, and cumulative counts. Emit a concise final counter summary at shutdown where lifecycle access is safe.
   - Keep all existing parameter defaults and output field layouts unchanged. If a tolerance is configurable, add it as a backward-compatible parameter with a safe default and document it in both launch file and README.

5. Add deterministic regression tests and integrate them with ament.
   - Add `ament_cmake_gtest` as a test dependency in `package.xml` and guard test targets with `if(BUILD_TESTING)` in the top-level `CMakeLists.txt`.
   - Add queue tests for empty/full behavior, FIFO ordering, explicit wrap-around with a very small capacity, more than two full wraps, maximum-backlog rejection, reset, and a concurrent producer/consumer run of at least 100,000 uniquely numbered packets. The concurrent test must fail on duplicates, stale replay, loss when not intentionally full, or ordering changes.
   - Add XT sequence tests for decoding each supported packet size, normal increment, gap counting, duplicate rejection, backward rejection, uint32 rollover, and restart-after-idle recovery.
   - Add timestamp-guard tests for monotonic input, permitted jitter, an approximately 7-second regression, recovery/reset behavior, and cloud-level regression rejection.
   - Keep the legacy SDK manual PCAP example intact unless it directly conflicts; do not mislabel it as an automated test.

6. Build and run offline verification in the driver repository.
   - In a ROS 2 Humble environment, run a clean package-scoped build using `colcon build --packages-select hesai_lidar --symlink-install --cmake-args -DBUILD_TESTING=ON` from an appropriate temporary or existing workspace that does not overwrite user artifacts.
   - Run `colcon test --packages-select hesai_lidar --event-handlers console_direct+` and `colcon test-result --verbose`; require zero failures.
   - Run focused tests repeatedly enough to cross many queue wraps. If practical, build and run the queue concurrency test under ThreadSanitizer separately; record tool/runtime limitations rather than treating an unavailable sanitizer as success.
   - Run syntax/static checks for modified launch and Python files and inspect compiler warnings from newly owned code. Do not hide warnings in the new queue/helper code behind the vendored SDK's existing `-w` blanket without documenting the limitation.

7. Validate the data path with `go2w-experiment-recorder` as the first canary.
   - Create a semantic consumer branch such as `fix/update-hesai-stale-packet-fix` after rechecking it is clean.
   - Temporarily point its Hesai submodule at the tested driver commit and rebuild the same container/ROS path used for actual experiments.
   - Add or use a durable PointCloud2 timestamp validator that checks message-header monotonicity and the per-point `timestamp` field, rather than relying only on rosbag storage timestamps. Record counts, maximum backward delta, and any queue/sequence diagnostic events.
   - Run a live XT16 dual-return recording long enough to traverse the 36,000-packet boundary many times; target at least 30 minutes. Require zero approximately-7-second packet/point/cloud regressions, zero stale replay detections, monotonic cloud headers within tolerance, and an expected packet/point rate.
   - Verify shutdown remains prompt and does not hang with the producer active or during an overload condition.
   - Preserve the bag and machine-readable validation report outside Git; do not commit bulky recordings.

8. Validate the downstream LIO contract with `dlio-go2w`.
   - On a clean semantic branch, update only the Hesai submodule pointer to the tested driver commit and rebuild the relevant ROS packages/image.
   - Run an offline known-good bag smoke test if a suitable uncorrupted bag/PCAP exists, then a live XT16+D-LIO run on the physical Jetson/Go2W when available.
   - Require `/points_raw` type, fields, frame, rate, and timestamps to remain compatible; D-LIO must remain running without time-regression warnings, transform failures caused by the change, or new point-cloud drops under normal load.
   - Keep Jetson and physical-robot status explicitly UNVERIFIED until actually run.

9. Commit and publish the shared driver fix.
   - Review the driver diff for scope and license preservation.
   - Commit the queue fix, defenses, tests, and documentation in intentional commits on `fix/prevent-stale-packet-replay` and push the branch to `koki67/go2w-hesai-lidar-driver`.
   - Open/use the repository's normal review path if one exists. Merge to `main` only after required offline gates pass and hardware gates are either completed or explicitly accepted by the user as deferred.
   - Record the immutable merged/full commit SHA. Add a release tag only if this repository already has a release convention or the user explicitly chooses one; do not invent versioning during the bug fix.

10. Roll out the immutable driver commit to direct submodule consumers.
    - In `go2w-experiment-recorder`, `dlio-go2w`, `slam-go2w`, and `wlo-go2w`, create/reuse semantic fix branches, fetch the driver commit, check out that exact commit in `humble_ws/src/go2w-hesai-lidar-driver`, stage the gitlink, and update any documented pinned SHA.
    - Correct the malformed `.gitmodules` URLs in `slam-go2w` and `wlo-go2w` to `https://github.com/koki67/go2w-hesai-lidar-driver.git`, run `git submodule sync`, and verify a fresh recursive clone resolves the same commit.
    - Run each repository's build/doctor/smoke commands. Commit and push each parent update separately so consumers can trace the exact driver provenance.
    - Do not claim the driver update has propagated merely because its `main` moved; verify each parent gitlink contains the new full SHA.

11. Update manifest/vendor consumers without losing their local integration patches.
    - Update `/home/user/ws/frontier-next` only after fetching its one missing remote commit and resolving any changed state non-destructively.
    - Change `dependencies.repos` to the new full driver SHA.
    - Rebase or regenerate `vendor_patches/go2w_hesai_lidar_driver/<base-sha>/0001-make-configured-correction-authoritative.patch` against the new driver base while preserving the patch's intended behavior.
    - Update `vendor_patches/manifest.yaml`, patch path/digest/base commit, and the commit-specific evidence URL in `config/dependency-policy.yaml`; preserve both Apache-2.0 license artifacts and hashes.
    - Run `scripts/verify_vendor_patches.py`, `vendor_patches/check.sh`, the repository's relevant tests, and a clean `vcs import`/patch-application check. Update only the owning `frontier-next` branch; do not mechanically edit every worktree branch. Rebase/merge the owner change into active worktrees through normal Git workflow as they continue.
    - Compare `/home/user/ws/nav-frontier-go2w-v3/humble_ws/src/hesai_lidar` against both the old driver commit and its parent-repository modifications. Port the fix by updating the vendored copy or replacing it with an explicit dependency only after preserving nav-specific changes. Build and smoke-test that repository before committing.

12. Perform final provenance and acceptance audit.
    - Search `/home/user/ws` for `koki67/go2w-hesai-lidar-driver`, the old SHA `6c6a93aa...`, copied `PacketsBuffer` implementations, and package `hesai_lidar`. Classify any remaining hits as intentionally archived, an active unupdated consumer, or an unrelated historical document.
    - Verify all intended parent branches/commits are pushed and every active consumer resolves to the fixed immutable SHA or contains an equivalent reviewed vendored patch.
    - Re-run the recorder timestamp report and D-LIO smoke result after the final parent pins, not only against a temporary checkout.
    - Report exact repositories/commits changed, tests passed, hardware duration, observed maximum timestamp regression, packet loss/gap counters, and anything still UNVERIFIED. Clean up branches only after merge confirmation and only according to each repository's workflow.

## Validation

- Driver repository:
  - `source /opt/ros/humble/setup.bash`
  - `colcon build --packages-select hesai_lidar --symlink-install --cmake-args -DBUILD_TESTING=ON`
  - `colcon test --packages-select hesai_lidar --event-handlers console_direct+`
  - `colcon test-result --verbose`
  - Repeated focused queue wrap/concurrency tests; optional ThreadSanitizer run recorded separately.
- Static/runtime contract checks:
  - `ros2 pkg executables hesai_lidar`
  - `ros2 topic type /points_raw` must remain `sensor_msgs/msg/PointCloud2`.
  - Inspect PointCloud2 fields, frame ID, rate, and QoS before/after.
  - Verify all launch files parse and retain existing defaults.
- Recorder canary:
  - At least 30 minutes of XT16 dual-return live capture when hardware is available.
  - Machine-readable header and point-level timestamp report with zero approximately-7-second regressions.
  - Queue/sequence/drop diagnostic totals saved with the run.
  - Prompt clean shutdown.
- D-LIO canary:
  - Offline known-good replay when available.
  - Live Jetson/Go2W smoke run when available; no D-LIO time regression or new TF/data-contract failure.
- Consumer rollout:
  - `git submodule status` shows the fixed full SHA in all four direct submodule consumers.
  - Fresh recursive clone check for corrected `slam-go2w` and `wlo-go2w` URLs.
  - `frontier-next` vendor-patch verifier and clean import/apply checks pass.
  - `nav-frontier-go2w-v3` build confirms the synchronized vendored source.
  - Final `rg` inventory finds no unexplained active reference to the old SHA or unsafe `PacketsBuffer` implementation.

## Acceptance criteria

- The unsafe iterator-based `PacketsBuffer` and its check/read/advance race no longer exist in the shared driver.
- Automated tests deterministically exercise multiple wraps and concurrent producer/consumer operation without stale replay, duplication, reordering, or unintended loss.
- XT sequence rollover, gaps, duplicates, backward packets, and restart recovery are covered by tests and produce the intended diagnostics/drop behavior.
- A packet or cloud timestamp approximately 7 seconds older than the accepted stream is rejected and reported before publication to downstream consumers.
- Public ROS topic names, package/executable names, frame behavior, launch defaults, PointCloud2 layout, and normal data rate remain compatible.
- Offline build and test gates pass with zero failures.
- When hardware is available, at least 30 minutes of XT16 dual-return capture traverses many former ring boundaries with no approximately-7-second header or point regressions.
- D-LIO consumes the fixed stream without new runtime, timestamp, TF, or point-field failures.
- Every confirmed active consumer is pinned to the fixed immutable commit or contains a reviewed equivalent vendored update; malformed submodule URLs are corrected.
- All hardware, Jetson, long-duration, merge, and rollout claims are labeled accurately as verified or UNVERIFIED.

## Risks / cautions

- The root cause is high-confidence from code and timing correspondence, but physical confirmation still requires a live XT16 run. Do not describe the field defect as fully resolved before that gate.
- Incorrect atomic memory ordering can create a subtler stale-read defect. Keep queue ownership strictly SPSC and rely on acquire/release semantics plus concurrency tests.
- Dropping on overload changes behavior from producer backpressure. The intended backlog limit is already 400 packets; instrument drops and verify normal operation does not reach it.
- Sequence numbers can reset when the LiDAR reboots and can wrap naturally. A naive strictly increasing comparison would permanently reject valid data; rollover and restart handling are mandatory.
- Host realtime can legitimately step due to clock synchronization. Timestamp regression tolerance and logging must distinguish small jitter from the multi-second defect, and any large clock step should fail safe for SLAM rather than be hidden.
- A cloud can contain a stale point even if its header is current. The packet-level guard is required; cloud-only reordering or header checks are insufficient.
- Existing bags are not repaired by the driver change and may still reproduce the old anomaly during replay.
- The `frontier-next` driver patch is content-addressed to its base SHA; changing only `dependencies.repos` will break verification. Regenerate all coupled metadata and validate patch application.
- `nav-frontier-go2w-v3` may contain source changes not present in the standalone driver. Do not overwrite the vendored directory without a three-way comparison.
- Updating many repositories creates provenance and branch-management risk. Use one immutable driver SHA, separate parent commits, and do not update dirty worktrees or force-push.
- Hardware, network, ROS middleware, and container access may be unavailable in the implementation environment. Complete all safe offline work, request only necessary access, and report blocked physical gates rather than substituting topic presence or successful compilation.
