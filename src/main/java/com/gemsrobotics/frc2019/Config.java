package com.gemsrobotics.frc2019;

import com.moandjiezana.toml.Toml;

import java.nio.file.Path;
import java.util.List;
import java.util.Optional;

@SuppressWarnings({"unused", "WeakerAccess"})
public class Config {
	private static final String CONFIG_DIR = "/home/lvuser/deploy/%s_config.toml";

	public static Toml getConfig(final String name) {
		return Optional.of(name)
					   .map(n -> String.format(CONFIG_DIR, n))
					   .map(Path::of)
					   .map(Path::toFile)
					   .map(f -> new Toml().read(f))
					   .orElseGet(Toml::new);
	}
}
