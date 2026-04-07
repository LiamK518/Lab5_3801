function [R] = rotation_matrix(attitude)
  c_phi   = cos(attitude(1));
  c_theta = cos(attitude(2));
  c_psi   = cos(attitude(3));

  s_phi   = sin(attitude(1));
  s_theta = sin(attitude(2));
  s_psi   = sin(attitude(3));

  R = [
    c_phi * c_psi, (s_phi * s_theta * c_psi) - (c_phi * s_psi), (c_phi * s_theta * c_psi) + (s_phi * s_psi);
    c_phi * s_psi, (s_phi * s_theta * s_psi) + (c_phi * c_psi), (c_phi * s_theta * s_psi) - (s_phi * c_psi);
    -s_theta, s_phi * c_theta, c_phi * c_theta;
  ];
end
