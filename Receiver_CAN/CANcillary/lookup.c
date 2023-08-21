
unsigned int lookup_u16_u16(unsigned int X_value, volatile const unsigned int * p_X_axis, volatile const unsigned int * p_Y_axis, unsigned int breakpoints)
{
	unsigned int breakpoint, breakpoint_next, difference;

	if(0 == breakpoints) return(0);

	breakpoint = breakpoints - 1;

	if(X_value >= p_X_axis[breakpoint])
	{
		return(p_Y_axis[breakpoint]);
	}
	else if(X_value <= p_X_axis[0])
	{
		return(p_Y_axis[0]);
	}
	else
	{
		while(p_X_axis[breakpoint - 1] > X_value)
		{
			breakpoint--;
		}

		breakpoint_next = breakpoint - 1;
		
		difference = (unsigned int)((0xFFFFUL * (unsigned long)(X_value - p_X_axis[breakpoint_next])) / (unsigned long)(p_X_axis[breakpoint] - p_X_axis[breakpoint_next]) + 1);
		
		if(p_Y_axis[breakpoint_next] < p_Y_axis[breakpoint])
		{
			return((unsigned int)((((unsigned long)difference * (unsigned long)(p_Y_axis[breakpoint] - p_Y_axis[breakpoint_next])) / 0xFFFFUL) + p_Y_axis[breakpoint_next]));
		}
		else
		{
			return(p_Y_axis[breakpoint_next] - (unsigned int)(((unsigned long)difference * (unsigned long)(p_Y_axis[breakpoint_next] - p_Y_axis[breakpoint])) / 0xFFFFUL));
		}		
	}
}


int lookup_u16_s16(unsigned int X_value, volatile const unsigned int * p_X_axis, volatile const int * p_Y_axis, unsigned int breakpoints)
{
	unsigned int breakpoint, breakpoint_next, difference;

	if(0 == breakpoints) return(0);	// protection

	breakpoint = breakpoints - 1;	// start at the top, m_Site must be >=1

	if(X_value >= p_X_axis[breakpoint])	// overflow shortcut
	{
		return(p_Y_axis[breakpoint]);
	}
	else if(X_value <= p_X_axis[0])	// underflow shortcut
	{
		return(p_Y_axis[0]);
	}
	else
	{
		while(p_X_axis[breakpoint - 1] > X_value)
		{
			breakpoint--;
		}

		breakpoint_next = breakpoint - 1;

		difference = (int)((0x7FFFL * (unsigned long)(X_value - p_X_axis[breakpoint_next])) / (unsigned long)(p_X_axis[breakpoint] - p_X_axis[breakpoint_next]) + 1);

		return((int)((((long)difference * (long)(p_Y_axis[breakpoint] - p_Y_axis[breakpoint_next])) / 0x7FFFL) + p_Y_axis[breakpoint_next]));
	}
}


int lookup_s16_s16(int X_value, volatile const int * p_X_axis, volatile const int * p_Y_axis, unsigned int breakpoints)
{
	int difference;
	unsigned int breakpoint, breakpoint_next;

	if(0 == breakpoints) return(0);

	breakpoint = breakpoints - 1;

	if(X_value >= p_X_axis[breakpoint])
	{
		return(p_Y_axis[breakpoint]);
	}
	else if(X_value <= p_X_axis[0])
	{
		return(p_Y_axis[0]);
	}
	else
	{
		while(p_X_axis[breakpoint - 1] > X_value)
		{
			breakpoint--;
		}

		breakpoint_next = breakpoint - 1;

		difference = (int)((0x7FFFL * (unsigned long)(X_value - p_X_axis[breakpoint_next])) / (unsigned long)(p_X_axis[breakpoint] - p_X_axis[breakpoint_next]) + 1);

		return((int)((((long)difference * (long)(p_Y_axis[breakpoint] - p_Y_axis[breakpoint_next])) / 0x7FFFL) + p_Y_axis[breakpoint_next]));
	}
}
