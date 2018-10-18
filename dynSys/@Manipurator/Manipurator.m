classdef Manipurator < DynSys
  properties
    dims
    u_max
    d_max
  end
  
  methods
    function obj = Manipurator(x, u_max, d_max, dims)
      
      if numel(x) ~= obj.nx
        error('Initial state does not have right dimension!');
      end
      
      if ~iscolumn(x)
        x = x';
      end
      
      if nargin < 2
        u_max = 1;
      end
      
      if nargin < 3
        d_max = 1;
      end
      
      if nargin < 4
        dims = 1:2;
      end
      
      % Basic vehicle properties
      obj.pdim = find(dims == 1); % Position dimensions
      obj.hdim = find(dims == 2);   % velosity dimensions
      obj.nx = length(dims);
      obj.nu = 1;
      obj.nd = 1;
      
      %obj.x = x;
      obj.xhist = obj.x;
      
      obj.u_max = u_max;
      obj.d_max = d_max;
      obj.dims = dims;
    end
    
  end % end methods
end % end classdef
