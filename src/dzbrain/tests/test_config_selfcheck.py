"""Input failures stay NO-GO; valid-input behaviour must not change.
Run: python -B -X utf8 src/dzbrain/tests/test_config_selfcheck.py
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

WS = Path(__file__).resolve().parents[3]
TOOL = WS / 'tools' / 'config_selfcheck.py'
BASELINE = WS.parent / '_tmp_session' / 'AGC-RECTIFY-20261026' / 'baseline' / 'car_files' / 'tools' / 'config_selfcheck.py'
CONFIG = WS / 'config' / 'robot.yaml'
FIXTURE = WS / 'src' / 'dzbrain' / 'tests' / 'fixtures' / 'map_sixth_20261017.yaml'


def run_tool(config, map_path, quiet=False, source=None, extra=()):
    env = dict(os.environ, PYTHONUTF8='1', PYTHONIOENCODING='utf-8', PYTHONDONTWRITEBYTECODE='1')
    args = ['--config', str(config), '--map', str(map_path)] + (['--quiet'] if quiet else []) + list(extra)
    source = source or os.environ.get('CONFIG_SELFCHECK_UNDER_TEST')
    if source:
        # Execute captured bytes with the real tool identity, keeping imports/defaults equal.
        runner = "import pathlib,sys; src=sys.argv.pop(1); target=sys.argv.pop(1); sys.argv[0]=target; exec(compile(pathlib.Path(src).read_text(encoding='utf-8'),target,'exec'),{'__name__':'__main__','__file__':target})"
        command = [sys.executable, '-B', '-X', 'utf8', '-c', runner, str(source), str(TOOL), *args]
    else:
        command = [sys.executable, '-B', '-X', 'utf8', str(TOOL), *args]
    return subprocess.run(command, cwd=str(WS), env=env, capture_output=True, text=True, encoding='utf-8', timeout=30)


class InputFailureTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def assert_no_go(self, result, expected):
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertNotIn('Traceback', result.stdout + result.stderr)
        rows = [line.split('=', 1)[1] for line in result.stdout.splitlines() if line.startswith('RESULT_JSON=')]
        self.assertEqual(len(rows), 1, result.stdout + result.stderr)
        data = json.loads(rows[0])
        self.assertIs(data['ok'], False)
        self.assertIs(data['launch_go'], False)
        self.assertEqual(data['status'], 'NO-GO')
        self.assertEqual(data['route_status'], 'UNKNOWN')
        self.assertEqual(data['error'], expected)

    def assert_not_launch_go(self, result, expected_status):
        """exit 3：配置可用但**空间证据未校验** —— 仍绝不是发车放行。"""
        self.assertEqual(result.returncode, 3, result.stderr)
        self.assertNotIn('Traceback', result.stdout + result.stderr)
        rows = [line.split('=', 1)[1] for line in result.stdout.splitlines() if line.startswith('RESULT_JSON=')]
        self.assertEqual(len(rows), 1, result.stdout + result.stderr)
        data = json.loads(rows[0])
        self.assertIs(data['launch_go'], False)
        self.assertEqual(data['status'], expected_status)
        self.assertEqual(data['route_status'], 'UNKNOWN')
        self.assertIsNone(data['error'])

    # ── 约定变更（2026-10-26 · AGC-RECTIFY-20261026）────────────────────────
    # 原断言：缺图 ⇒ MAP_INPUT_ERROR（exit 1）。改因：车端 layout 模式下车载图恒不存在
    # （start_all.sh 的 --map 指向不存在的文件，main.py::_layout_mode_enabled 判为布局模式），
    # 若仍判 input error，则放行门慢速预检**恒红**、真实问题被假红淹没。
    # 意图不变：布局模式仍然 launch_go=false / route_status=UNKNOWN，**绝不放行**。
    # 反向守卫保留：auto_plan_enabled=false 时缺图仍是真缺输入 ⇒ MAP_INPUT_ERROR。
    def test_missing_map_in_layout_mode_is_not_input_error(self):
        self.assert_not_launch_go(run_tool(CONFIG, self.root / 'absent.yaml'), 'LAYOUT-MODE')

    def test_missing_map_in_layout_mode_quiet_still_not_launch_go(self):
        self.assert_not_launch_go(run_tool(CONFIG, self.root / 'absent.yaml', quiet=True), 'LAYOUT-MODE')

    def test_missing_map_without_autoplan_stays_input_error(self):
        """auto_plan_enabled=false ⇒ 缺图是真缺输入，必须仍然判红（exit 1）。"""
        text = CONFIG.read_text(encoding='utf-8')
        self.assertIn('auto_plan_enabled: true', text,
                      'fixture 前提失效：robot.yaml 里找不到 auto_plan_enabled: true')
        patched = self.root / 'robot_noautoplan.yaml'
        patched.write_text(text.replace('auto_plan_enabled: true', 'auto_plan_enabled: false', 1),
                           encoding='utf-8')
        self.assert_no_go(run_tool(patched, self.root / 'absent.yaml'), 'MAP_INPUT_ERROR')

    def test_missing_config(self):
        self.assert_no_go(run_tool(self.root / 'absent.yaml', FIXTURE), 'CONFIG_INPUT_ERROR')

    def test_bad_map_yaml(self):
        path = self.root / 'bad.yaml'
        path.write_text('map: [unterminated', encoding='utf-8')
        self.assert_no_go(run_tool(CONFIG, path), 'MAP_INPUT_ERROR')

    def test_non_mapping_map(self):
        path = self.root / 'bad.yaml'
        path.write_text('- A1\n- B2\n', encoding='utf-8')
        self.assert_no_go(run_tool(CONFIG, path), 'MAP_INPUT_ERROR')

    def test_non_utf8_map(self):
        path = self.root / 'bad.yaml'
        path.write_bytes(bytes([255, 254, 128]))
        self.assert_no_go(run_tool(CONFIG, path), 'MAP_INPUT_ERROR')

    def test_bad_config_yaml(self):
        path = self.root / 'bad.yaml'
        path.write_text('/dzbrain: [unterminated', encoding='utf-8')
        self.assert_no_go(run_tool(path, FIXTURE), 'CONFIG_INPUT_ERROR')

    def test_empty_or_false_map_is_not_spatial_evidence(self):
        for text in ('', 'false\n', '0\n', '{}\n', 'map: {}\n'):
            with self.subTest(text=text):
                path = self.root / 'empty_map.yaml'
                path.write_text(text, encoding='utf-8')
                self.assert_no_go(run_tool(CONFIG, path), 'MAP_INPUT_ERROR')

    def test_bad_nested_map_rejected_in_normal_and_quiet_modes(self):
        for text in ('map: null\n', 'map: []\n', 'map: wrong\n'):
            for quiet in (False, True):
                with self.subTest(text=text, quiet=quiet):
                    path = self.root / 'nested_map.yaml'
                    path.write_text(text, encoding='utf-8')
                    self.assert_no_go(run_tool(CONFIG, path, quiet=quiet), 'MAP_INPUT_ERROR')

    def test_incomplete_map_is_not_spatial_evidence(self):
        path = self.root / 'incomplete_map.yaml'
        path.write_text('map: {name: missing_world, start: N10, end: N5}\n', encoding='utf-8')
        self.assert_no_go(run_tool(CONFIG, path), 'MAP_INPUT_ERROR')

    def test_map_collections_cannot_silently_mean_empty(self):
        import yaml
        for key in ('mines', 'obstacles', 'targets', 'bridges'):
            with self.subTest(key=key):
                data = yaml.safe_load(FIXTURE.read_text(encoding='utf-8'))
                data['map'][key] = None
                path = self.root / 'bad_collection.yaml'
                path.write_text(yaml.safe_dump(data, allow_unicode=True), encoding='utf-8')
                self.assert_no_go(run_tool(CONFIG, path), 'MAP_INPUT_ERROR')

    def test_non_mapping_or_empty_config(self):
        for text in ('- item\n', 'wrong\n', '42\n', '', '{}\n'):
            with self.subTest(text=text):
                path = self.root / 'bad_config.yaml'
                path.write_text(text, encoding='utf-8')
                self.assert_no_go(run_tool(path, FIXTURE), 'CONFIG_INPUT_ERROR')

    def test_out_of_range_config_is_reported_without_traceback(self):
        import yaml
        data = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
        data['/dzbrain']['ros__parameters']['pursuit']['cruise_speed_mps'] = -1.0
        path = self.root / 'out_of_range.yaml'
        path.write_text(yaml.safe_dump(data, allow_unicode=True), encoding='utf-8')
        self.assert_no_go(run_tool(path, FIXTURE), 'CONFIG_VALIDATION_ERROR')

    def test_flat_map_reuses_same_validation_as_nested_map(self):
        import yaml
        data = yaml.safe_load(FIXTURE.read_text(encoding='utf-8'))['map']
        path = self.root / 'flat_map.yaml'
        path.write_text(yaml.safe_dump(data, allow_unicode=True), encoding='utf-8')
        nested = run_tool(CONFIG, FIXTURE, quiet=True)
        flat = run_tool(CONFIG, path, quiet=True)
        self.assertEqual((flat.returncode, flat.stdout, flat.stderr), (nested.returncode, nested.stdout, nested.stderr))

    def test_nested_type_error_never_tracebacks(self):
        # 复核 F1：mission_blocks 被迁移成 route.tokens 后是 int，validate 里
        # `for tok in tokens` 抛 TypeError（core/config.py:859）——曾以 traceback 结束、无 RESULT_JSON。
        path = self.root / 'nested_type.yaml'
        path.write_text('path_executor:\n  mission_blocks: 5\n', encoding='utf-8')
        result = run_tool(path, FIXTURE)
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertNotIn('Traceback', result.stdout + result.stderr)
        self.assertEqual(result.stdout.count('RESULT_JSON='), 1, result.stdout)

    def test_empty_bridges_is_rejected_not_green(self):
        # 复核 F2：车端布局协议 allow_empty=False（tasks/drone.py:242），静态图空桥洞必须判错
        import yaml
        data = yaml.safe_load(FIXTURE.read_text(encoding='utf-8'))
        data['map']['bridges'] = []
        path = self.root / 'empty_bridges.yaml'
        path.write_text(yaml.safe_dump(data, allow_unicode=True), encoding='utf-8')
        self.assert_no_go(run_tool(CONFIG, path), 'MAP_INPUT_ERROR')

    def test_explicit_empty_feature_is_visible_not_plain_green(self):
        # 复核 F2：显式 mines:[] 不得静默变成 ✅ 通过——它是"本局无雷"的主张
        import yaml
        data = yaml.safe_load(FIXTURE.read_text(encoding='utf-8'))
        data['map']['mines'] = []
        path = self.root / 'empty_mines.yaml'
        path.write_text(yaml.safe_dump(data, allow_unicode=True), encoding='utf-8')
        result = run_tool(CONFIG, path)
        self.assertNotIn('Traceback', result.stdout + result.stderr)
        self.assertIn('[显式空集合]', result.stdout)
        self.assertNotIn('✅ 通过', result.stdout)
        self.assertIn('有条件通过', result.stdout)

    def test_config_only_is_not_a_launch_clearance(self):
        result = run_tool(CONFIG, FIXTURE, extra=('--config-only',))
        self.assertEqual(result.returncode, 3, result.stdout + result.stderr)
        self.assertNotIn('Traceback', result.stdout + result.stderr)
        rows = [line.split('=', 1)[1] for line in result.stdout.splitlines()
                if line.startswith('RESULT_JSON=')]
        self.assertEqual(len(rows), 1, result.stdout + result.stderr)
        data = json.loads(rows[0])
        self.assertIs(data['launch_go'], False)
        self.assertEqual(data['status'], 'CONFIG-ONLY')
        self.assertEqual(data['route_status'], 'UNKNOWN')
        self.assertIs(data['spatial_checked'], False)
        self.assertNotIn('✅ 通过', result.stdout)

    def test_config_only_still_rejects_bad_config(self):
        path = self.root / 'bad.yaml'
        path.write_text('- item\n', encoding='utf-8')
        result = run_tool(path, FIXTURE, extra=('--config-only',))
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assert_no_go(result, 'CONFIG_INPUT_ERROR')

    def test_fixture_behaviour_is_unchanged(self):
        # 先断言契约本身（不依赖 _tmp 基线副本），避免基线被清理后"静默 skip 变绿"（复核 F4）
        contract = run_tool(CONFIG, FIXTURE, quiet=True)
        self.assertEqual(contract.returncode, 0, contract.stdout + contract.stderr)
        self.assertIn('✅ 通过', contract.stdout)
        self.assertNotIn('Traceback', contract.stdout + contract.stderr)
        if not BASELINE.exists():
            self.skipTest('captured pre-change baseline is not installed')
        self.assertTrue(FIXTURE.exists(), 'reuse fixture must exist')
        before = run_tool(CONFIG, FIXTURE, quiet=True, source=BASELINE)
        after = run_tool(CONFIG, FIXTURE, quiet=True)
        self.assertEqual((after.returncode, after.stdout, after.stderr), (before.returncode, before.stdout, before.stderr))
        self.assertNotIn('Traceback', after.stdout + after.stderr)


if __name__ == '__main__':
    unittest.main(verbosity=2)
