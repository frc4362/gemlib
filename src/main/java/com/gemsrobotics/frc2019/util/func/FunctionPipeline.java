package com.gemsrobotics.frc2019.util.func;

import java.util.Arrays;
import java.util.List;
import java.util.function.Function;

/**
 * A class which packages a ton of functions that will
 * all be called in sequence on any data fed to it
 * @param <T> The type which will be processed
 */
@SuppressWarnings({"unused", "WeakerAccess"})
public final class FunctionPipeline<T> {
	private Function<T, T> m_func;

	/**
	 * @param functions The functions to be composed
	 */
	public FunctionPipeline(final List<Function<T, T>> functions) {
		// reduces every function into one
		m_func = functions.stream().reduce(Function.identity(), Function::andThen);
	}

	@SafeVarargs
	public FunctionPipeline(final Function<T, T>... functions) {
		this(Arrays.asList(functions));
	}

	public FunctionPipeline() {
		this(Function.identity());
	}

	/**
	 * Compose another function onto the end
	 * @param func The new function to add to the pipeline
	 * @return The function pipeline with the new function included
	 */
	public FunctionPipeline<T> map(final Function<T, T> func) {
		m_func = m_func.andThen(func);
		return this;
	}

	/**
	 * Applies every function to the specified input
	 */
	public T apply(T input) {
		return m_func.apply(input);
	}
}
