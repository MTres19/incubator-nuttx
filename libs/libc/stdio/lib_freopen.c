      /* Yes, open the file directly if no stream is opened yet */
      if (stream == NULL)
          return fopen(path, mode);
      /* Otherwise, open the file */
      oflags = lib_mode2oflags(mode);
      if (oflags < 0)
        {
          return NULL;
        }

      fd = open(path, oflags, 0666);
      if (fd < 0)
        {
          return NULL;
        }

      /* Flush the stream and duplicate the new fd to it */

      fflush(stream);
      ret = dup2(fd, fileno(stream));
      close(fd);
      if (ret < 0)
        {
          return NULL;
        }

      clearerr(stream);
      return stream;