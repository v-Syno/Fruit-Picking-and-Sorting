function status = GlobalEStop(flag)
    % GlobalEStop manages the global E-Stop state
    % If a flag is provided, it toggles the E-Stop state.
    % Otherwise, it returns the current state.
    
    persistent eStopActive % Persistent variable to hold E-Stop state
    if isempty(eStopActive)
        eStopActive = false; % Initialize as false
    end
    
    if nargin > 0
        % Toggle state if flag is provided
        eStopActive = flag;
    end
    
    status = eStopActive;
end