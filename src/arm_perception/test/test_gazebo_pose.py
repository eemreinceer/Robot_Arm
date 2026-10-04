from arm_perception.gazebo_pose import GazeboPose, parse_pose_info, pose_inside_bin_xy


def test_parse_pose_info_extracts_named_model_pose():
    text = '''
pose {
  name: "sorting_red_box_00"
  position {
    x: 0.46
    y: 0.16
    z: 0.625
  }
  orientation {
    x: 0
    y: 0
    z: 0
    w: 1
  }
}
pose {
  name: "bin_red"
  position {
    x: 0.59
    y: -0.31
    z: 0.6
  }
}
'''
    poses = parse_pose_info(text)

    assert poses['sorting_red_box_00'].xyz == (0.46, 0.16, 0.625)
    assert poses['sorting_red_box_00'].quaternion == (0.0, 0.0, 0.0, 1.0)
    assert poses['bin_red'].xyz == (0.59, -0.31, 0.6)
    assert poses['bin_red'].quaternion == (0.0, 0.0, 0.0, 1.0)


def test_pose_inside_bin_xy_uses_inner_bounds():
    bin_center = (0.59, -0.31, 0.60)
    inner_size = (0.16, 0.16, 0.08)

    assert pose_inside_bin_xy(
        GazeboPose('sorting_red_box_00', (0.51, -0.39, 0.64), (0.0, 0.0, 0.0, 1.0)),
        bin_center,
        inner_size,
    )
    assert not pose_inside_bin_xy(
        GazeboPose('sorting_red_box_00', (0.549, -0.446, 0.759), (0.0, 0.0, 0.0, 1.0)),
        bin_center,
        inner_size,
    )


def test_parse_pose_info_handles_proto3_omitted_zero_fields():
    """Statik bin regresyonu (2026-07-05): proto3 text çıktısı sıfır alanları
    yazmaz — z'siz position ve orientation'sız blok parse edilebilmeli."""
    text = '''
pose {
  name: "bin_red"
  id: 42
  position {
    x: -0.12
    y: -0.19
  }
}
pose {
  name: "at_origin"
  id: 43
}
'''
    poses = parse_pose_info(text)
    assert 'bin_red' in poses
    assert poses['bin_red'].xyz == (-0.12, -0.19, 0.0)
    assert poses['bin_red'].quaternion == (0.0, 0.0, 0.0, 1.0)
    assert 'at_origin' in poses
    assert poses['at_origin'].xyz == (0.0, 0.0, 0.0)
