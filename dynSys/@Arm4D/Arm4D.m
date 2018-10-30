classdef Arm4D < DynSys
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
    function obj = Arm4D(x, uMax, dMax, dims)
        
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
     

    end % end constructor

  end % end methods
end % end class