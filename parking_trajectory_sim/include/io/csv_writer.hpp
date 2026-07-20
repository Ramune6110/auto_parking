// Traceability: IMPL-CSV-01
#pragma once

#include "simulation/simulation_result.hpp"

#include <string>

/**
 * @brief 参照軌道と車両シミュレーション結果を2つのCSVへ保存する。
 * @param result 保存対象の軌道と全周期のシミュレーション結果。
 * @param output_directory CSVを作成するディレクトリ。
 * @param wheelbase 参照曲率から参照舵角を計算するホイールベース [m]。
 */
void writeSimulationCsvFiles(
    const SimulationResult& result,
    const std::string& output_directory,
    double wheelbase);
