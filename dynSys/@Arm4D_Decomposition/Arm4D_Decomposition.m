classdef Arm4D_Decomposition < DynSys
  properties
    dims    % Active dimensions
    uMax    % Control bounds
    dMax
    q_max   % Joint limits
    q_min
    dq_max  % Velocity limits
    dq_min
  end % end properties

  methods
    function obj = Arm4D_Decomposition(x, uMax, dMax, grid_min, grid_max, dims)
        
      if numel(x) ~= obj.nx
        error('Initial state does not have right dimension!');
      end
      
      if ~iscolumn(x)
        x = x';
      end
      
      % Default control bounds if not provided
      if nargin < 2
        uMax = 3;
      end
      
      if nargin < 3
        dMax = 1;
      end
      
      % Default number of dims if not provided
      if nargin < 6
        dims = 1:4;
      end
      
      obj.nx = length(dims);
      obj.nu = 2;
      obj.nd = 2;
      
      obj.x = x;
      obj.xhist = x;
      
      obj.uMax = uMax;
      obj.dMax = dMax;
      obj.dims = dims;
      
      % joint limits
      obj.q_min = grid_min(1:2); %[0; -pi/2];
      obj.q_max = grid_max(1:2); %[pi; pi/2];
      % veloicity limits
      obj.dq_min = grid_min(3:4); %[-20; -20];
      obj.dq_max = grid_max(3:4); %[20; 20];

    end % end constructor

  end % end methods
end % end class