/**
 * Swizzled DocSidebar Mobile Content
 * Purpose: Show docs sidebar items + auth section at bottom (mobile)
 * Date: 2025-12-18
 */

import React from 'react';
import clsx from 'clsx';
import { ThemeClassNames } from '@docusaurus/theme-common';
import { translate } from '@docusaurus/Translate';
import DocSidebarItems from '@theme/DocSidebarItems';
import { SidebarAuth } from '../../../../components/auth/SidebarAuth';
import styles from './styles.module.css';

// -----------------------------
// TypeScript-safe props
// -----------------------------
interface MobileContentProps {
  path: string;
  sidebar: any; // could refine with DocSidebarItem[] if needed
  className?: string;
}

// -----------------------------
// Component
// -----------------------------
export default function DocSidebarMobileContent({
  path,
  sidebar,
  className,
}: MobileContentProps): JSX.Element {
  return (
    <nav
      aria-label={translate({
        id: 'theme.docs.sidebar.navAriaLabel',
        message: 'Docs sidebar',
        description: 'The ARIA label for the sidebar navigation',
      })}
      className={clsx(
        ThemeClassNames.docs.docSidebarMenu,
        styles.menu,
        className
      )}
    >
     <ul className="menu__list">
  <DocSidebarItems items={sidebar} activePath={path} level={1} />
  <li className={styles.authWrapper}>
    <SidebarAuth />
  </li>
</ul>

    </nav>
  );
}
