classdef testGravityAccelerationKernels < matlab.unittest.TestCase
    %% DESCRIPTION
    % Unit tests for reusable point-mass and differential third-body gravity kernels.
    % Values use independent physical oracles. Analytical derivatives are also
    % checked against central finite differences over representative 3-D states.
    % -------------------------------------------------------------------------------------------------------------

    methods (Test)

        function testPointMassAccelerationMatchesPhysicalOracle(self)
            %% SIGNATURE
            % self.testPointMassAccelerationMatchesPhysicalOracle()
            %% DESCRIPTION
            % Verify point mass acceleration matches physical oracle.
            %% INPUT
            % self   MATLAB assertion context.
            %% OUTPUT
            % None; assert the gravity interface contract.
            %% CHANGELOG
            % 15-09-2026  Pietro Califano  Add the isolated gravity regression.
            % 11-10-2026  Pietro Califano  Consolidate under the current owner.
            %% DEPENDENCIES
            % ComputePointMassGravity, EvalJac_PointMassGravity,
            % ComputeThirdBodyGravity, EvalJac_ThirdBodyGravity

            dPositionFromBody = [7200.0; -450.0; 220.0];
            dBodyGM = 398600.4418;

            dActual = ComputePointMassGravity(dPositionFromBody, dBodyGM);
            dExpected = -dBodyGM .* dPositionFromBody ./ norm(dPositionFromBody).^3;

            self.verifyEqual(dActual, dExpected, 'RelTol', 1.0e-14);
            self.verifyLessThan(dot(dActual, dPositionFromBody), 0.0);
        end

        function testPointMassJacobianMatchesOracleAndFiniteDifference(self)
            %% SIGNATURE
            % self.testPointMassJacobianMatchesOracleAndFiniteDifference()
            %% DESCRIPTION
            % Verify point mass jacobian matches oracle and finite difference.
            %% INPUT
            % self   MATLAB assertion context.
            %% OUTPUT
            % None; assert the gravity interface contract.
            %% CHANGELOG
            % 15-09-2026  Pietro Califano  Add the isolated gravity regression.
            % 11-10-2026  Pietro Califano  Consolidate under the current owner.
            %% DEPENDENCIES
            % ComputePointMassGravity, EvalJac_PointMassGravity,
            % ComputeThirdBodyGravity, EvalJac_ThirdBodyGravity

            dPositionFromBody = [7200.0; -450.0; 220.0];
            dBodyGM = 398600.4418;
            dDistance = norm(dPositionFromBody);
            dExpected = dBodyGM .* ( ...
                3.0 .* (dPositionFromBody * transpose(dPositionFromBody)) ./ dDistance.^5 - ...
                eye(3) ./ dDistance.^3);

            dActual = EvalJac_PointMassGravity(dPositionFromBody, dBodyGM);
            dFiniteDifference = testGravityAccelerationKernels.finiteDifference_( ...
                @(dPosition) ComputePointMassGravity(dPosition, dBodyGM), ...
                dPositionFromBody);

            self.verifyEqual(dActual, dExpected, 'RelTol', 1.0e-14, 'AbsTol', 1.0e-18);
            self.verifyEqual(dActual, dFiniteDifference, 'RelTol', 1.0e-8, 'AbsTol', 1.0e-13);
        end

        function testThirdBodyAccelerationMatchesPhysicalOracle(self)
            %% SIGNATURE
            % self.testThirdBodyAccelerationMatchesPhysicalOracle()
            %% DESCRIPTION
            % Verify third body acceleration matches physical oracle.
            %% INPUT
            % self   MATLAB assertion context.
            %% OUTPUT
            % None; assert the gravity interface contract.
            %% CHANGELOG
            % 15-09-2026  Pietro Califano  Add the isolated gravity regression.
            % 11-10-2026  Pietro Califano  Consolidate under the current owner.
            %% DEPENDENCIES
            % ComputePointMassGravity, EvalJac_PointMassGravity,
            % ComputeThirdBodyGravity, EvalJac_ThirdBodyGravity

            dSpacecraftPosition = [1.0; 0.2; -0.1];
            dBodyPosition = [10.0; -2.0; 1.0];
            dBodyGM = 100.0;
            dBodyFromSpacecraft = dBodyPosition - dSpacecraftPosition;
            dExpected = dBodyGM .* ( ...
                dBodyFromSpacecraft ./ norm(dBodyFromSpacecraft).^3 - ...
                dBodyPosition ./ norm(dBodyPosition).^3);

            dActual = ComputeThirdBodyGravity( ...
                dSpacecraftPosition, dBodyPosition, dBodyGM);

            self.verifyEqual(dActual, dExpected, 'RelTol', 1.0e-14, 'AbsTol', 1.0e-14);
        end

        function testThirdBodyFarFieldRetainsTidalAcceleration(self)
            %% SIGNATURE
            % self.testThirdBodyFarFieldRetainsTidalAcceleration()
            %% DESCRIPTION
            % Verify third body far field retains tidal acceleration.
            %% INPUT
            % self   MATLAB assertion context.
            %% OUTPUT
            % None; assert the gravity interface contract.
            %% CHANGELOG
            % 15-09-2026  Pietro Califano  Add the isolated gravity regression.
            % 11-10-2026  Pietro Califano  Consolidate under the current owner.
            %% DEPENDENCIES
            % ComputePointMassGravity, EvalJac_PointMassGravity,
            % ComputeThirdBodyGravity, EvalJac_ThirdBodyGravity

            % Itokawa-scale target-relative displacement and one-AU Sun range [km].
            dSpacecraftPosition = [2.0; 0.0; 0.0];
            dBodyPosition = [149597870.7; 0.0; 0.0];
            dBodyGM = 132712440041.9394;
            dSpacecraftDistance = dSpacecraftPosition(1);
            dBodyDistance = dBodyPosition(1);

            % Closed-form collinear difference avoids subtracting two nearly equal accelerations.
            dExpectedAxial = dBodyGM * dSpacecraftDistance * ...
                (2.0 * dBodyDistance - dSpacecraftDistance) / ...
                (dBodyDistance.^2 * (dBodyDistance - dSpacecraftDistance).^2);
            dActual = ComputeThirdBodyGravity( ...
                dSpacecraftPosition, dBodyPosition, dBodyGM);
            dExpectedJacobian = dBodyGM / (dBodyDistance - dSpacecraftDistance).^3 .* ...
                diag([2.0, -1.0, -1.0]);
            dActualJacobian = EvalJac_ThirdBodyGravity( ...
                dSpacecraftPosition, dBodyPosition, dBodyGM);

            self.verifyEqual(dActual(1), dExpectedAxial, 'RelTol', 1.0e-13);
            self.verifyEqual(dActual(2:3), zeros(2, 1), 'AbsTol', 0.0);
            self.verifyEqual(dActualJacobian, dExpectedJacobian, ...
                'RelTol', 1.0e-14, 'AbsTol', 1.0e-30);
        end

        function testThirdBodyJacobianMatchesOracleAndFiniteDifference(self)
            %% SIGNATURE
            % self.testThirdBodyJacobianMatchesOracleAndFiniteDifference()
            %% DESCRIPTION
            % Verify third body jacobian matches oracle and finite difference.
            %% INPUT
            % self   MATLAB assertion context.
            %% OUTPUT
            % None; assert the gravity interface contract.
            %% CHANGELOG
            % 15-09-2026  Pietro Califano  Add the isolated gravity regression.
            % 11-10-2026  Pietro Califano  Consolidate under the current owner.
            %% DEPENDENCIES
            % ComputePointMassGravity, EvalJac_PointMassGravity,
            % ComputeThirdBodyGravity, EvalJac_ThirdBodyGravity

            dSpacecraftPosition = [1.0; 0.2; -0.1];
            dBodyPosition = [10.0; -2.0; 1.0];
            dBodyGM = 100.0;
            dBodyFromSpacecraft = dBodyPosition - dSpacecraftPosition;
            dDistance = norm(dBodyFromSpacecraft);
            dExpected = dBodyGM .* ( ...
                3.0 .* (dBodyFromSpacecraft * transpose(dBodyFromSpacecraft)) ./ dDistance.^5 - ...
                eye(3) ./ dDistance.^3);

            dActual = EvalJac_ThirdBodyGravity( ...
                dSpacecraftPosition, dBodyPosition, dBodyGM);
            dFiniteDifference = testGravityAccelerationKernels.finiteDifference_( ...
                @(dPosition) ComputeThirdBodyGravity(dPosition, dBodyPosition, dBodyGM), ...
                dSpacecraftPosition);

            self.verifyEqual(dActual, dExpected, 'RelTol', 1.0e-14, 'AbsTol', 1.0e-14);
            self.verifyEqual(dActual, dFiniteDifference, 'RelTol', 1.0e-8, 'AbsTol', 1.0e-10);
        end

        function testZeroGravitationalParameterReturnsExactZeros(self)
            %% SIGNATURE
            % self.testZeroGravitationalParameterReturnsExactZeros()
            %% DESCRIPTION
            % Verify zero gravitational parameter returns exact zeros.
            %% INPUT
            % self   MATLAB assertion context.
            %% OUTPUT
            % None; assert the gravity interface contract.
            %% CHANGELOG
            % 15-09-2026  Pietro Califano  Add the isolated gravity regression.
            % 11-10-2026  Pietro Califano  Consolidate under the current owner.
            %% DEPENDENCIES
            % ComputePointMassGravity, EvalJac_PointMassGravity,
            % ComputeThirdBodyGravity, EvalJac_ThirdBodyGravity

            dSpacecraftPosition = [1.0; 0.2; -0.1];
            dBodyPosition = [10.0; -2.0; 1.0];

            self.verifyEqual( ...
                ComputePointMassGravity(dSpacecraftPosition, 0.0), zeros(3, 1), 'AbsTol', 0.0);
            self.verifyEqual( ...
                EvalJac_PointMassGravity(dSpacecraftPosition, 0.0), zeros(3, 3), 'AbsTol', 0.0);
            self.verifyEqual( ...
                ComputeThirdBodyGravity(dSpacecraftPosition, dBodyPosition, 0.0), ...
                zeros(3, 1), 'AbsTol', 0.0);
            self.verifyEqual( ...
                EvalJac_ThirdBodyGravity(dSpacecraftPosition, dBodyPosition, 0.0), ...
                zeros(3, 3), 'AbsTol', 0.0);
        end

        function testGravitySingularitiesAreRejected(self)
            %% SIGNATURE
            % self.testGravitySingularitiesAreRejected()
            %% DESCRIPTION
            % Verify gravity singularities are rejected.
            %% INPUT
            % self   MATLAB assertion context.
            %% OUTPUT
            % None; assert the gravity interface contract.
            %% CHANGELOG
            % 15-09-2026  Pietro Califano  Add the isolated gravity regression.
            % 11-10-2026  Pietro Califano  Consolidate under the current owner.
            %% DEPENDENCIES
            % ComputePointMassGravity, EvalJac_PointMassGravity,
            % ComputeThirdBodyGravity, EvalJac_ThirdBodyGravity

            self.verifyError(@() ComputePointMassGravity(zeros(3, 1), 1.0), ...
                'ComputePointMassGravity:ZeroDistance');
            self.verifyError(@() EvalJac_PointMassGravity(zeros(3, 1), 1.0), ...
                'EvalJac_PointMassGravity:ZeroDistance');
            self.verifyError(@() ComputeThirdBodyGravity([1; 0; 0], zeros(3, 1), 1.0), ...
                'ComputeThirdBodyGravity:ZeroMainBodyDistance');
            self.verifyError(@() ComputeThirdBodyGravity([1; 0; 0], [1; 0; 0], 1.0), ...
                'ComputeThirdBodyGravity:ZeroSpacecraftDistance');
            self.verifyError(@() EvalJac_ThirdBodyGravity([1; 0; 0], [1; 0; 0], 1.0), ...
                'EvalJac_ThirdBodyGravity:ZeroSpacecraftDistance');
            self.verifyError(@() EvalJac_ThirdBodyGravity([1; 0; 0], zeros(3, 1), 1.0), ...
                'EvalJac_ThirdBodyGravity:ZeroMainBodyDistance');
        end

    end

    methods (Static, Access = private)

        function dJacobian = finiteDifference_(fAcceleration, dPosition)
            dStep = 1.0e-6 * max(1.0, norm(dPosition));
            dJacobian = zeros(3, 3);
            for idAxis = 1:3
                dPerturbation = zeros(3, 1);
                dPerturbation(idAxis) = dStep;
                dAccelerationPlus = fAcceleration(dPosition + dPerturbation);
                dAccelerationMinus = fAcceleration(dPosition - dPerturbation);
                dJacobian(:, idAxis) = ...
                    (dAccelerationPlus - dAccelerationMinus) / (2.0 * dStep);
            end
        end

    end
end
