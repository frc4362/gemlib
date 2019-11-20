package com.gemsrobotics.lib;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;

@SuppressWarnings({"unused"})
public class FileObjects {
	private static final Gson CONFIG_READER = new GsonBuilder().create();
	private static final String CONFIG_DIR = "/home/lvuser/deploy/%s.json";

	public static <T> T get(final Class<T> klass, final String name) {
        return Optional.of(name)
                .map(n -> String.format(CONFIG_DIR, n))
                .map(Path::of)
                .flatMap(FileObjects::readLines)
                .map(lines -> String.join("", lines))
                .map(raw -> CONFIG_READER.fromJson(raw, klass))
                .orElse(null);
	}

	private static Optional<List<String>> readLines(final Path linesToRead) {
		try {
			return Optional.of(Files.readAllLines(linesToRead));
		} catch (final IOException ioException) {
			return Optional.empty();
		}
	}
}
