/*
 * This file is part of Placeholder-2023, licensed under the GNU General Public License (GPLv3).
 *
 * Copyright (c) Octobots <https://github.com/Octobots9084>
 * Copyright (c) contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

package frc.robot.Commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Subsystems.drivetrain.SwerveDrive;
import frc.robot.Subsystems.light.Animations;
import frc.robot.Subsystems.light.Light;

public class BalanceChargeStationAuto extends PIDCommand {
    public BalanceChargeStationAuto() {
        // Light.getInstance().setAnimation(Animations.BALANCING);

        super(
                new PIDController(0.015, 0.000, 0),
                SwerveDrive.getInstance()::getPitch,
                0,
                output -> SwerveDrive.getInstance().drive(new ChassisSpeeds(output, 0, 0), false),
                SwerveDrive.getInstance());

        getController().setTolerance(20, 20);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
