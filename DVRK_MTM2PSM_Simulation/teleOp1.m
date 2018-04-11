classdef teleOp < handle
  properties(Access = public)

    publisher;
    jointStateMsg;
    tStart = tic;
    dt = 0.001;
    % add your code
  end

    methods(Access = public)
        
        function obj = teleOp(publisher,jointStateMsg)  % Constructor

            if (nargin > 1)
                obj.publisher = publisher;
                obj.jointStateMsg = jointStateMsg;
            end
            % add your code
        end

        function  [psm_q,tracking_err] = run(obj, mtm_q)
          
            % add your code below

        end    
        
        function  callback_update_mtm_q(obj,q)
            obj.jointStateMsg.Position = obj.run(q);
            tElapsed = toc(obj.tStart);
            if (tElapsed > 0.033)
                obj.tStart = tic;
                obj.publisher.send(obj.jointStateMsg);
            end    
        end

    end
end
