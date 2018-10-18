function dx = dynamics(obj, ~, x, u, d)
% dx = dynamics(obj, t, x, u, d)

if nargin < 5
  d = 0;
end

if iscell(x)
  dx = cell(length(obj.dims), 1);
  
  for i = 1:length(obj.dims)
    dx{i} = dynamics_cell_helper(obj, x, u, d, obj.dims, obj.dims(i));
  end
else
  dx = zeros(obj.nx, 1);
  
  dx(1) = x(2);
  dx(2) = u + d;
end
end

function dx = dynamics_cell_helper(obj, x, u, d, dims, dim)

switch dim
  case 1
    dx = x{dims==2};
  case 2
    dx = u + d;
  otherwise
    error('Only dimension 1-2 are defined for dynamics of Manipurator!')
end
end