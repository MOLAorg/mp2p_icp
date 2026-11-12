"""Tests of error reporting: clean messages from C++ (MRPT) exceptions, and
validation of the structure of pipeline configurations."""

from __future__ import annotations

import mrpt.maps
import pytest

import mp2p_icp


def test_mrpt_exceptions_have_clean_messages():
    with pytest.raises(RuntimeError) as info:
        mp2p_icp.filter_pipeline_from_yaml([{"class_name": "NoSuchFilter", "params": {}}])
    msg = str(info.value)
    assert msg == "Cannot create object of class `NoSuchFilter`, is it registered?"
    # The full C++ text, with the location and call stack:
    details = info.value.cpp_traceback
    assert "==== MRPT exception ====" in details
    assert "Location:" in details and "NoSuchFilter" in details

    icp, _ = mp2p_icp.icp_pipeline_from_yaml(mp2p_icp.config_path("2d-point2point"))
    with pytest.raises(RuntimeError) as info:
        icp.align(mp2p_icp.MetricMap(), mp2p_icp.MetricMap())
    assert "Call stack" not in str(info.value)
    assert "pcGlobal.empty()" in str(info.value)


def test_other_exceptions_unchanged(tmp_path):
    with pytest.raises(FileNotFoundError) as info:
        mp2p_icp.MetricMap.from_file(tmp_path / "missing.mm")
    assert not hasattr(info.value, "cpp_traceback")


@pytest.mark.parametrize(
    "cfg, match",
    [
        ({"filters": [1, 2]}, "entry #1 must be a map with 'class_name' and 'params', got '1'"),
        ([{"params": {}}], "entry #1 has no 'class_name'"),
        ([{"class_name": "mp2p_icp_filters::FilterDecimateVoxels"}], "entry #1 has no 'params'"),
        ([{"class_name": "mp2p_icp_filters::FilterDecimateVoxels", "params": 3}], "must be a map"),
        ({"filters": "x"}, "expected a list of entries"),
    ],
)
def test_filter_pipeline_validation(cfg, match):
    with pytest.raises(ValueError, match=match):
        mp2p_icp.filter_pipeline_from_yaml(cfg)


def test_generators_validation():
    with pytest.raises(ValueError, match="Invalid generators configuration: entry #2"):
        mp2p_icp.generators_from_yaml(
            [{"class_name": "mp2p_icp_filters::Generator", "params": None}, [1]]
        )


def test_icp_pipeline_validation():
    base = mp2p_icp.load_yaml(mp2p_icp.config_path("2d-point2point"))
    mp2p_icp.ICP.from_yaml(base)  # valid

    with pytest.raises(ValueError, match="expected a map"):
        mp2p_icp.ICP.from_yaml([base])
    for key, match in [
        ("class_name", "missing 'class_name'"),
        ("params", "missing the 'params' map"),
        ("quality", "missing the 'quality' list"),
    ]:
        cfg = {k: v for k, v in base.items() if k != key}
        with pytest.raises(ValueError, match=match):
            mp2p_icp.ICP.from_yaml(cfg)
    with pytest.raises(ValueError, match="'matchers' configuration: entry #1 has no 'class'"):
        mp2p_icp.ICP.from_yaml(dict(base, matchers=[{"params": {}}]))
    with pytest.raises(ValueError, match="'solvers' configuration: expected a list"):
        mp2p_icp.ICP.from_yaml(dict(base, solvers={"class": "mp2p_icp::Solver_Horn"}))
    # Disabled entries are ignored, as in C++:
    mp2p_icp.ICP.from_yaml(dict(base, matchers=base["matchers"] + [{"enabled": False}]))


def test_sm2mm_pipeline_validation():
    with pytest.raises(ValueError, match="'final_filters' configuration: entry #1"):
        mp2p_icp.simplemap_to_metricmap(mrpt.maps.CSimpleMap(), {"final_filters": [None]})
    with pytest.raises(ValueError, match="expected a map"):
        mp2p_icp.simplemap_to_metricmap(mrpt.maps.CSimpleMap(), [1])
