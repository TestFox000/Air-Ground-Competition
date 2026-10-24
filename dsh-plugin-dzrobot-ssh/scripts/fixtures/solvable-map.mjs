/**
 * 测试夹具入口 —— 「引擎可解地图」的路径与四类点位。
 *
 * 只被 scripts/test-sim-*.mjs 引用，运行时（sim.html / index.js / src/*.js）不引用本文件。
 * 背景与几何来源见同目录 map_fifth_solvable.yaml 顶部注释（第六张图 tracker-safe 无解，
 * 同引擎 E2E 需要一个引擎真能解的稳定图）。
 */
import path from 'node:path'
import { fileURLToPath } from 'node:url'

export const FIXTURE_MAP = path.join(path.dirname(fileURLToPath(import.meta.url)), 'map_fifth_solvable.yaml')

/** 与 FIXTURE_MAP 的 map: 段逐格一致（truthPlanEvidence 的 board 入参用）。 */
export const FIXTURE = {
  mines: ['C1', 'A3', 'E3', 'G4', 'I6', 'B7', 'D9', 'H10'],
  obstacles: ['A1', 'E2', 'H4', 'D8', 'I8', 'G9', 'B11', 'F11'],
  targets: ['E5', 'F5'],
  bridges: ['K5', 'K9'],
}
