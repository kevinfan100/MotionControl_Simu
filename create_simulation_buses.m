function create_simulation_buses()
%CREATE_SIMULATION_BUSES Create Bus Objects for Simulink simulation
%
%   create_simulation_buses()
%
%   Creates nested Bus Objects in the base workspace for use with Simulink.
%   This function must be called before running the Simulink model.
%
%   Bus structure:
%       ParamsBus
%       ├── common (CommonBus)
%       ├── wall (WallBus)
%       ├── traj (TrajBus)
%       ├── ctrl (CtrlBus)
%       └── thermal (ThermalBus)
%
%   Note: String parameters are encoded as numbers for Simulink compatibility:
%       - traj.type: 0='z_move', 1='xy_circle'
%       - traj.direction: 0='away', 1='toward'

    %% CommonBus
    elems = Simulink.BusElement.empty(0, 4);

    elems(1) = Simulink.BusElement;
    elems(1).Name = 'R';
    elems(1).Dimensions = [1 1];
    elems(1).DimensionsMode = 'Fixed';
    elems(1).DataType = 'double';

    elems(2) = Simulink.BusElement;
    elems(2).Name = 'gamma_N';
    elems(2).Dimensions = [1 1];
    elems(2).DimensionsMode = 'Fixed';
    elems(2).DataType = 'double';

    elems(3) = Simulink.BusElement;
    elems(3).Name = 'Ts';
    elems(3).Dimensions = [1 1];
    elems(3).DimensionsMode = 'Fixed';
    elems(3).DataType = 'double';

    elems(4) = Simulink.BusElement;
    elems(4).Name = 'T_sim';
    elems(4).Dimensions = [1 1];
    elems(4).DimensionsMode = 'Fixed';
    elems(4).DataType = 'double';

    CommonBus = Simulink.Bus;
    CommonBus.Elements = elems;
    assignin('base', 'CommonBus', CommonBus);

    %% WallBus
    elems = Simulink.BusElement.empty(0, 7);

    elems(1) = Simulink.BusElement;
    elems(1).Name = 'theta';
    elems(1).Dimensions = [1 1];
    elems(1).DimensionsMode = 'Fixed';
    elems(1).DataType = 'double';

    elems(2) = Simulink.BusElement;
    elems(2).Name = 'phi';
    elems(2).Dimensions = [1 1];
    elems(2).DimensionsMode = 'Fixed';
    elems(2).DataType = 'double';

    elems(3) = Simulink.BusElement;
    elems(3).Name = 'pz';
    elems(3).Dimensions = [1 1];
    elems(3).DimensionsMode = 'Fixed';
    elems(3).DataType = 'double';

    elems(4) = Simulink.BusElement;
    elems(4).Name = 'h_bar_min';
    elems(4).Dimensions = [1 1];
    elems(4).DimensionsMode = 'Fixed';
    elems(4).DataType = 'double';

    elems(5) = Simulink.BusElement;
    elems(5).Name = 'w_hat';
    elems(5).Dimensions = [3 1];
    elems(5).DimensionsMode = 'Fixed';
    elems(5).DataType = 'double';

    elems(6) = Simulink.BusElement;
    elems(6).Name = 'u_hat';
    elems(6).Dimensions = [3 1];
    elems(6).DimensionsMode = 'Fixed';
    elems(6).DataType = 'double';

    elems(7) = Simulink.BusElement;
    elems(7).Name = 'v_hat';
    elems(7).Dimensions = [3 1];
    elems(7).DimensionsMode = 'Fixed';
    elems(7).DataType = 'double';

    WallBus = Simulink.Bus;
    WallBus.Elements = elems;
    assignin('base', 'WallBus', WallBus);

    %% TrajBus
    elems = Simulink.BusElement.empty(0, 8);

    elems(1) = Simulink.BusElement;
    elems(1).Name = 'type';  % 0=z_move, 1=xy_circle
    elems(1).Dimensions = [1 1];
    elems(1).DimensionsMode = 'Fixed';
    elems(1).DataType = 'double';

    elems(2) = Simulink.BusElement;
    elems(2).Name = 'h_margin';
    elems(2).Dimensions = [1 1];
    elems(2).DimensionsMode = 'Fixed';
    elems(2).DataType = 'double';

    elems(3) = Simulink.BusElement;
    elems(3).Name = 'delta_z';
    elems(3).Dimensions = [1 1];
    elems(3).DimensionsMode = 'Fixed';
    elems(3).DataType = 'double';

    elems(4) = Simulink.BusElement;
    elems(4).Name = 'direction';  % 0=away, 1=toward
    elems(4).Dimensions = [1 1];
    elems(4).DimensionsMode = 'Fixed';
    elems(4).DataType = 'double';

    elems(5) = Simulink.BusElement;
    elems(5).Name = 'speed';
    elems(5).Dimensions = [1 1];
    elems(5).DimensionsMode = 'Fixed';
    elems(5).DataType = 'double';

    elems(6) = Simulink.BusElement;
    elems(6).Name = 'radius';
    elems(6).Dimensions = [1 1];
    elems(6).DimensionsMode = 'Fixed';
    elems(6).DataType = 'double';

    elems(7) = Simulink.BusElement;
    elems(7).Name = 'period';
    elems(7).Dimensions = [1 1];
    elems(7).DimensionsMode = 'Fixed';
    elems(7).DataType = 'double';

    elems(8) = Simulink.BusElement;
    elems(8).Name = 'n_circles';
    elems(8).Dimensions = [1 1];
    elems(8).DimensionsMode = 'Fixed';
    elems(8).DataType = 'double';

    TrajBus = Simulink.Bus;
    TrajBus.Elements = elems;
    assignin('base', 'TrajBus', TrajBus);

    %% CtrlBus
    elems = Simulink.BusElement.empty(0, 4);

    elems(1) = Simulink.BusElement;
    elems(1).Name = 'enable';  % 0=off, 1=on
    elems(1).Dimensions = [1 1];
    elems(1).DimensionsMode = 'Fixed';
    elems(1).DataType = 'double';

    elems(2) = Simulink.BusElement;
    elems(2).Name = 'lambda_c';
    elems(2).Dimensions = [1 1];
    elems(2).DimensionsMode = 'Fixed';
    elems(2).DataType = 'double';

    elems(3) = Simulink.BusElement;
    elems(3).Name = 'gamma';
    elems(3).Dimensions = [1 1];
    elems(3).DimensionsMode = 'Fixed';
    elems(3).DataType = 'double';

    elems(4) = Simulink.BusElement;
    elems(4).Name = 'Ts';
    elems(4).Dimensions = [1 1];
    elems(4).DimensionsMode = 'Fixed';
    elems(4).DataType = 'double';

    CtrlBus = Simulink.Bus;
    CtrlBus.Elements = elems;
    assignin('base', 'CtrlBus', CtrlBus);

    %% ThermalBus
    elems = Simulink.BusElement.empty(0, 5);

    elems(1) = Simulink.BusElement;
    elems(1).Name = 'enable';  % 0=off, 1=on
    elems(1).Dimensions = [1 1];
    elems(1).DimensionsMode = 'Fixed';
    elems(1).DataType = 'double';

    elems(2) = Simulink.BusElement;
    elems(2).Name = 'k_B';
    elems(2).Dimensions = [1 1];
    elems(2).DimensionsMode = 'Fixed';
    elems(2).DataType = 'double';

    elems(3) = Simulink.BusElement;
    elems(3).Name = 'T';
    elems(3).Dimensions = [1 1];
    elems(3).DimensionsMode = 'Fixed';
    elems(3).DataType = 'double';

    elems(4) = Simulink.BusElement;
    elems(4).Name = 'Ts';
    elems(4).Dimensions = [1 1];
    elems(4).DimensionsMode = 'Fixed';
    elems(4).DataType = 'double';

    elems(5) = Simulink.BusElement;
    elems(5).Name = 'variance_coeff';
    elems(5).Dimensions = [1 1];
    elems(5).DimensionsMode = 'Fixed';
    elems(5).DataType = 'double';

    ThermalBus = Simulink.Bus;
    ThermalBus.Elements = elems;
    assignin('base', 'ThermalBus', ThermalBus);

    %% ParamsBus (parent bus)
    elems = Simulink.BusElement.empty(0, 5);

    elems(1) = Simulink.BusElement;
    elems(1).Name = 'common';
    elems(1).Dimensions = [1 1];
    elems(1).DimensionsMode = 'Fixed';
    elems(1).DataType = 'Bus: CommonBus';

    elems(2) = Simulink.BusElement;
    elems(2).Name = 'wall';
    elems(2).Dimensions = [1 1];
    elems(2).DimensionsMode = 'Fixed';
    elems(2).DataType = 'Bus: WallBus';

    elems(3) = Simulink.BusElement;
    elems(3).Name = 'traj';
    elems(3).Dimensions = [1 1];
    elems(3).DimensionsMode = 'Fixed';
    elems(3).DataType = 'Bus: TrajBus';

    elems(4) = Simulink.BusElement;
    elems(4).Name = 'ctrl';
    elems(4).Dimensions = [1 1];
    elems(4).DimensionsMode = 'Fixed';
    elems(4).DataType = 'Bus: CtrlBus';

    elems(5) = Simulink.BusElement;
    elems(5).Name = 'thermal';
    elems(5).Dimensions = [1 1];
    elems(5).DimensionsMode = 'Fixed';
    elems(5).DataType = 'Bus: ThermalBus';

    ParamsBus = Simulink.Bus;
    ParamsBus.Elements = elems;
    assignin('base', 'ParamsBus', ParamsBus);

    fprintf('Bus Objects created successfully in base workspace.\n');
end
